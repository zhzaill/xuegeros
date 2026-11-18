# 资产来源与许可记录

> 目的：任何二进制进入本仓库，必须能回答"从哪来、能不能分发"。缺任一项，就只能本地留档，不能进入可推送历史。
> 状态：2026-11-18 建档。**本文件不代表任何许可声明，只记录事实与待确认项。**

## 1. `yolo11n.pt`（仓库根，5,613,764 B）

| 项 | 值 |
|---|---|
| SHA-256 | `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1` |
| 字节数 | 5,613,764 |
| 对应官方资产 | `https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt` |
| 比对结果 | 该地址 `HEAD` 返回 `200`，`Content-Length` = **5,613,764**，与本文件**逐字节一致** |
| 判定 | 来源**已确认**为 Ultralytics YOLO11n 官方权重（同尺寸；未做哈希对官方清单的二次核对） |
| 许可 | 上游为 **AGPL-3.0**（Ultralytics 默认条款）。**分发**该文件即触发 AGPL 的相应义务；本项目若公开，需要随附许可证文本并明确来源，或不分发该文件 |
| 在项目中的角色 | **非必需**：README 已写明 YOLO 已从 VLA 感知路径移除，适配器默认 `XUEGEROS_YOLO_ENABLE=0`。保留它只为调试便利 |
| 建议 | 分发前二选一：①随附 AGPL 文本与来源链接后保留；②从可推送历史中排除，改为运行时按需下载。**不因"它是权重文件"而默认删除** |

## 2. 车模 STL（`sim/xuegeros_sim/meshes/`，9 个文件，约 28.6 MB）

| 项 | 值 |
|---|---|
| 来源 | 本车自己的 CAD 导出，与 guest 上 `xuegecar_description/meshes/` 同源 |
| 字节数与 SHA-256 | 逐文件列在 `sim/xuegeros_sim/meshes/SOURCE.md` |
| 许可 | **未确立**。`xuegecar_description` 未提供 LICENSE 文件，车辆/固件提供方也未给出书面条款 |
| 判定 | 可以本地留档、可以在本机与自有服务器上使用；**在许可确立前不得进入公开/可分发的历史** |

## 3. 历史暂停决策（已被下面的独立源码发布线取代）

- 当前 `origin` 停在 `bf9be08`（2026-10-24），本机领先 22 个提交。
- `90f2beb` 已把 9 个 STL 写入历史。**仅在后续提交中删除文件，旧提交仍会随推送一并分发**；因此若选择排除，必须在推送前重写提交图（例如用 `git filter-branch`/`git filter-repo` 从历史中剔除 `sim/xuegeros_sim/meshes/*.stl`，或另起一条不含该提交的发布历史）。
- `yolo11n.pt` 在 `origin` 上**已经存在**（属于更早的提交），同样受上述"旧提交仍会分发"的约束。
- 结论：**在两项许可明确之前，不推送。** 备份走 git bundle；发布走单独准备的干净历史。

## 4. 2026-11-18 独立发布线

用户已要求公开仓库并发布 npm。npm 0.3.3 采用显式源码清单，不附 STL 或模型权重。
公开源码以旧 origin/master 为基线增加一个独立提交，避免引入本地含 STL 的研发历史；
本地完整 master 继续保留资产和研发记录，不合并进公开发布线，也不强推改写 origin。
旧 origin 已有 yolo11n.pt，随附 LICENSES/AGPL-3.0.txt 与本来源记录，未作 MIT 再许可。
上游源码与权重地址：https://github.com/ultralytics/ultralytics 、
https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt 。
尺寸匹配只能证明尺寸一致；尚未独立比对官方哈希，不能据此声称字节内容一致。
STL 仍仅本地使用，公开安装用程序化模型，用户可自行用 --mesh-dir 指定有权使用的 CAD。
