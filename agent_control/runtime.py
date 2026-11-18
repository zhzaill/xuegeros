"""Small process-level guards shared by the mapping flow and tests."""

from __future__ import annotations

import os
import subprocess


def mapping_controller_running(domain_id: str | None = None) -> bool:
    """Return true for an installed mapping controller in the active ROS domain."""
    active_domain = str(domain_id if domain_id is not None else os.environ.get("ROS_DOMAIN_ID", "0"))
    try:
        result = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
    except OSError:
        return False
    for line in result.stdout.splitlines():
        fields = line.strip().split(None, 1)
        if len(fields) != 2 or "agent_control" not in fields[1] or "mapping_controller" not in fields[1]:
            continue
        if "xuegeros_agent_flow.py" in fields[1]:
            continue
        try:
            with open(f"/proc/{fields[0]}/environ", "rb") as handle:
                process_env = handle.read().split(b"\0")
            values = {item.split(b"=", 1)[0].decode(): item.split(b"=", 1)[1].decode() for item in process_env if b"=" in item}
        except (OSError, ValueError):
            values = {}
        if values.get("ROS_DOMAIN_ID", "0") == active_domain:
            return True
    return False
