"""
vla_server.py — NoMaD inference service for XuegerOS (server side).

Faithful port of the official deployment loop
(visualnav-transformer: deployment/src/navigate.py + pd_controller.py),
stripped of ROS and matplotlib, exposed as HTTP:

  GET  /health        service status, weights provenance, VRAM, latency of last call
  POST /goal          {"image": "<b64 jpeg>"}            set persistent goal image
  POST /reset         clear the rolling observation context
  POST /infer         {"image": "<b64 jpeg>", "goal_image"?: "<b64 jpeg>",
                       "num_samples"?: 4, "waypoint"?: 2}
                      -> waypoints (robot frame), chosen waypoint, (v, w) proposal

The output is a PROPOSAL only. Nothing here talks to ROS or the car; the
XuegerOS motion gateway still owns every physical pulse (lease, clamps,
watchdog). Scale conventions mirror navigate.py; final calibration happens in
the shadow phase (P1), not here.

Run: E:\\ComfyUI\\venv\\Scripts\\python.exe E:\\vla\\nomad\\vla_server.py
Env: VLA_PORT (18300), VLA_MAX_V (0.08), VLA_MAX_W (0.4), VLA_RATE (5),
     VLA_NUM_SAMPLES (4), VLA_WAYPOINT (2), VLA_SCALE_MODE (raw|max_v_rate)
"""

import base64
import hashlib
import io
import json
import logging
import os
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image as PILImage
from torchvision import transforms

SERVER_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SERVER_DIR)
sys.path.insert(0, os.path.join(os.path.dirname(SERVER_DIR), "pydeps"))

from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from vint_train.models.nomad.nomad import DenseNetwork, NoMaD
from vint_train.models.nomad.nomad_vint import NoMaD_ViNT, replace_bn_with_gn

# --- official nomad.yaml (published checkpoint params) ----------------------

MODEL_PARAMS = {
    "model_type": "nomad",
    "vision_encoder": "nomad_vint",
    "encoding_size": 256,
    "obs_encoder": "efficientnet-b0",
    "mha_num_attention_heads": 4,
    "mha_num_attention_layers": 4,
    "mha_ff_dim_factor": 4,
    "down_dims": [64, 128, 256],
    "cond_predict_scale": False,
    "num_diffusion_iters": 10,
    "len_traj_pred": 8,
    "normalize": True,
    "image_size": [96, 96],
    "context_size": 3,
}

# train/vint_train/data/data_config.yaml
ACTION_STATS = {
    "min": np.array([-2.5, -4]),
    "max": np.array([5, 4]),
}

WEIGHTS_PATH = os.path.join(SERVER_DIR, "weights", "nomad.pth")

MAX_V = float(os.environ.get("VLA_MAX_V", "0.08"))
MAX_W = float(os.environ.get("VLA_MAX_W", "0.4"))
RATE = float(os.environ.get("VLA_RATE", "5"))
DT = 1.0 / RATE
NUM_SAMPLES = int(os.environ.get("VLA_NUM_SAMPLES", "4"))
WAYPOINT_IDX = int(os.environ.get("VLA_WAYPOINT", "2"))
SCALE_MODE = os.environ.get("VLA_SCALE_MODE", "raw")  # raw = get_action output only
PORT = int(os.environ.get("VLA_PORT", "18300"))

# fp32 first deployment: Pascal + tiny 96x96 inputs; correctness over speed.
# fp16 is a measured optimization TODO once real weights are in.
DTYPE = torch.float32

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    handlers=[
        logging.FileHandler(os.path.join(SERVER_DIR, "vla_server.log"), encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger("vla")

# --- model -------------------------------------------------------------------


def build_model(device):
    vision_encoder = NoMaD_ViNT(
        obs_encoding_size=MODEL_PARAMS["encoding_size"],
        context_size=MODEL_PARAMS["context_size"],
        mha_num_attention_heads=MODEL_PARAMS["mha_num_attention_heads"],
        mha_num_attention_layers=MODEL_PARAMS["mha_num_attention_layers"],
        mha_ff_dim_factor=MODEL_PARAMS["mha_ff_dim_factor"],
    )
    vision_encoder = replace_bn_with_gn(vision_encoder)
    noise_pred_net = ConditionalUnet1D(
        input_dim=2,
        global_cond_dim=MODEL_PARAMS["encoding_size"],
        down_dims=MODEL_PARAMS["down_dims"],
        cond_predict_scale=MODEL_PARAMS["cond_predict_scale"],
    )
    dist_pred_network = DenseNetwork(embedding_dim=MODEL_PARAMS["encoding_size"])
    return NoMaD(
        vision_encoder=vision_encoder,
        noise_pred_net=noise_pred_net,
        dist_pred_net=dist_pred_network,
    ).to(device=device, dtype=DTYPE)


def load_weights(model, device):
    """Returns weights info dict. Missing file -> random init (explicitly flagged)."""
    if not os.path.exists(WEIGHTS_PATH):
        log.warning("weights MISSING at %s — running random init, proposals are garbage", WEIGHTS_PATH)
        return {"loaded": False, "reason": "weights_file_missing", "path": WEIGHTS_PATH}
    raw = open(WEIGHTS_PATH, "rb").read()
    digest = hashlib.sha256(raw).hexdigest()
    checkpoint = torch.load(io.BytesIO(raw), map_location=device)
    result = model.load_state_dict(checkpoint, strict=False)
    return {
        "loaded": True,
        "path": WEIGHTS_PATH,
        "size_bytes": len(raw),
        "sha256": digest,
        "missing_keys": len(result.missing_keys),
        "unexpected_keys": len(result.unexpected_keys),
    }


DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
MODEL = build_model(DEVICE)
WEIGHTS_INFO = load_weights(MODEL, DEVICE)
MODEL.eval()
NOISE_SCHEDULER = DDPMScheduler(
    num_train_timesteps=MODEL_PARAMS["num_diffusion_iters"],
    beta_schedule="squaredcos_cap_v2",
    clip_sample=True,
    prediction_type="epsilon",
)
LOCK = threading.Lock()
CONTEXT = deque(maxlen=MODEL_PARAMS["context_size"] + 1)
GOAL_IMG = {"img": None, "set_at": None}
LAST_LATENCY = {"ms": None}


def transform_images(pil_imgs, image_size, center_crop=False):
    """deployment/src/utils.py transform_images, verbatim minus unused branches."""
    transform_type = transforms.Compose(
        [
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )
    if not isinstance(pil_imgs, list):
        pil_imgs = [pil_imgs]
    transf_imgs = []
    for pil_img in pil_imgs:
        pil_img = pil_img.resize(image_size)
        transf_img = transform_type(pil_img)
        transf_imgs.append(transf_img.unsqueeze(0))
    return torch.cat(transf_imgs, dim=1)


def unnormalize_data(ndata, stats):
    ndata = (ndata + 1) / 2
    return ndata * (stats["max"] - stats["min"]) + stats["min"]


def get_action(diffusion_output):
    """train_utils.get_action, extracted (cumsum of unnormalized deltas)."""
    ndeltas = diffusion_output.reshape(diffusion_output.shape[0], -1, 2)
    ndeltas = ndeltas.detach().float().cpu().numpy()
    actions = np.cumsum(unnormalize_data(ndeltas, ACTION_STATS), axis=1)
    return actions


def pd_controller(waypoint):
    """deployment/src/pd_controller.py, clamped to XuegerOS gateway limits."""
    dx, dy = float(waypoint[0]), float(waypoint[1])
    eps = 1e-8
    if abs(dx) < eps:
        v = 0.0
        w = np.sign(dy) * np.pi / (2 * DT) if abs(dy) > eps else 0.0
    else:
        v = dx / DT
        w = np.arctan(dy / dx) / DT
    return float(np.clip(v, 0, MAX_V)), float(np.clip(w, -MAX_W, MAX_W))


def run_inference(frame, goal_img, num_samples, waypoint_idx):
    obs_images = transform_images(list(CONTEXT), MODEL_PARAMS["image_size"])
    obs_images = torch.cat(torch.split(obs_images, 3, dim=1), dim=1).to(DEVICE, DTYPE)
    goal_image = transform_images(goal_img, MODEL_PARAMS["image_size"]).to(DEVICE, DTYPE)
    mask = torch.zeros(1).long().to(DEVICE)

    with torch.no_grad():
        obsgoal_cond = MODEL(
            "vision_encoder",
            obs_img=obs_images,
            goal_img=goal_image,
            input_goal_mask=mask,
        )
        dist_pred = MODEL("dist_pred_net", obsgoal_cond=obsgoal_cond)
        dist = float(dist_pred.flatten()[0])

        obs_cond = obsgoal_cond[0].unsqueeze(0)
        if len(obs_cond.shape) == 2:
            obs_cond = obs_cond.repeat(num_samples, 1)
        else:
            obs_cond = obs_cond.repeat(num_samples, 1, 1)
        naction = torch.randn(
            (num_samples, MODEL_PARAMS["len_traj_pred"], 2), device=DEVICE, dtype=DTYPE
        )
        NOISE_SCHEDULER.set_timesteps(MODEL_PARAMS["num_diffusion_iters"])
        for k in NOISE_SCHEDULER.timesteps[:]:
            noise_pred = MODEL(
                "noise_pred_net", sample=naction, timestep=k, global_cond=obs_cond
            )
            naction = NOISE_SCHEDULER.step(
                model_output=noise_pred, timestep=k, sample=naction
            ).prev_sample

    actions = get_action(naction)  # (num_samples, len_traj_pred, 2), robot frame
    chosen = actions[0][waypoint_idx]
    if SCALE_MODE == "max_v_rate":  # navigate.py line 226-227 convention
        chosen = chosen.copy()
        chosen[:2] *= MAX_V / RATE
    v, w = pd_controller(chosen)
    return {
        "waypoints_m": np.round(actions[0], 4).tolist(),
        "all_samples_first_waypoint": np.round(actions[:, 0], 4).tolist(),
        "chosen_waypoint": [round(float(chosen[0]), 4), round(float(chosen[1]), 4)],
        "velocity_proposal": {"v": round(v, 4), "w": round(w, 4)},
        "dist_pred": round(dist, 4),
        "scale_mode": SCALE_MODE,
        "num_samples": num_samples,
        "waypoint_index": waypoint_idx,
    }


# --- HTTP ---------------------------------------------------------------------


def b64_to_pil(data):
    raw = base64.b64decode(data)
    img = PILImage.open(io.BytesIO(raw))
    img.load()
    return img.convert("RGB")


class Handler(BaseHTTPRequestHandler):
    server_version = "xuegeros-vla/0.1"

    def _json(self, status, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("content-type", "application/json; charset=utf-8")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        log.info("%s %s", self.address_string(), fmt % args)

    def do_GET(self):
        if self.path != "/health":
            return self._json(404, {"ok": False, "error": "not_found"})
        vram = None
        try:
            vram = round(torch.cuda.memory_allocated() / 1e6, 1)
        except Exception:
            pass
        self._json(
            200,
            {
                "ok": True,
                "model": "nomad",
                "params": MODEL_PARAMS,
                "device": str(DEVICE),
                "dtype": str(DTYPE),
                "weights": WEIGHTS_INFO,
                "context_frames": len(CONTEXT),
                "context_required": MODEL_PARAMS["context_size"] + 1,
                "goal_set": GOAL_IMG["img"] is not None,
                "limits": {"max_v": MAX_V, "max_w": MAX_W, "rate": RATE},
                "vram_mb": vram,
                "last_latency_ms": LAST_LATENCY["ms"],
            },
        )

    def do_POST(self):
        length = int(self.headers.get("content-length") or 0)
        if length > 12 * 1024 * 1024:
            return self._json(413, {"ok": False, "error": "body_too_large"})
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except Exception as exc:
            return self._json(400, {"ok": False, "error": f"bad_json: {exc}"})

        if self.path == "/goal":
            try:
                GOAL_IMG["img"] = b64_to_pil(body["image"])
                GOAL_IMG["set_at"] = time.time()
            except Exception as exc:
                return self._json(400, {"ok": False, "error": f"bad_image: {exc}"})
            return self._json(200, {"ok": True, "goal_set": True})

        if self.path == "/reset":
            CONTEXT.clear()
            return self._json(200, {"ok": True, "context_cleared": True})

        if self.path != "/infer":
            return self._json(404, {"ok": False, "error": "not_found"})

        if not WEIGHTS_INFO.get("loaded"):
            log.warning("infer called without real weights (random init)")
        goal = None
        if body.get("goal_image"):
            try:
                goal = b64_to_pil(body["goal_image"])
                GOAL_IMG["img"] = goal
                GOAL_IMG["set_at"] = time.time()
            except Exception as exc:
                return self._json(400, {"ok": False, "error": f"bad_goal_image: {exc}"})
        elif GOAL_IMG["img"] is not None:
            goal = GOAL_IMG["img"]
        else:
            return self._json(400, {"ok": False, "error": "no_goal_image"})

        try:
            frame = b64_to_pil(body["image"])
        except Exception as exc:
            return self._json(400, {"ok": False, "error": f"bad_image: {exc}"})

        num_samples = int(body.get("num_samples") or NUM_SAMPLES)
        waypoint_idx = int(body.get("waypoint") or WAYPOINT_IDX)

        CONTEXT.append(frame)
        bootstrapped = False
        while len(CONTEXT) < MODEL_PARAMS["context_size"] + 1:
            CONTEXT.appendleft(frame)  # pragmatic bootstrap: repeat current frame
            bootstrapped = True

        started = time.time()
        try:
            with LOCK:
                result = run_inference(frame, goal, num_samples, waypoint_idx)
        except Exception as exc:
            log.exception("infer failed")
            return self._json(500, {"ok": False, "error": f"infer_failed: {exc}"})
        LAST_LATENCY["ms"] = round((time.time() - started) * 1000, 1)
        result.update(
            ok=True,
            weights_loaded=WEIGHTS_INFO.get("loaded", False),
            context_bootstrapped=bootstrapped,
            latency_ms=LAST_LATENCY["ms"],
        )
        return self._json(200, result)


def main():
    log.info(
        "vla_server starting: device=%s weights=%s port=%d limits v<=%.2f w<=%.2f",
        DEVICE,
        WEIGHTS_INFO.get("loaded"),
        PORT,
        MAX_V,
        MAX_W,
    )
    httpd = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
