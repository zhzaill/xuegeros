import { DEFAULTS, ID, LABEL } from './config.js';
import { XuegerosWorld } from './world.js';

export const XUEGEROS = {
  id: ID,
  label: LABEL,
  defaults: () => structuredClone(DEFAULTS),
  preflight: ({ cfg }) => {
    if (!cfg?.robotdUrl) throw new Error('XuegerOS robotdUrl 为空');
    if (!['physical', 'simulation'].includes(cfg?.deployment ?? 'physical')) {
      throw new Error('XuegerOS deployment 必须是 physical 或 simulation');
    }
    if (cfg?.deployment === 'simulation') {
      const url = new URL(String(cfg?.simulationRobotdUrl || ''));
      if (url.protocol !== 'ws:' || !url.hostname || url.username || url.password || url.hash) {
        throw new Error('XuegerOS simulationRobotdUrl 必须是 ws://<host>:<port>/...');
      }
    }
  },
  describe: {
    tools: [
      { name: 'xuegeros_control', summary: '读取或保存中控与小脑参数' },
      { name: 'xuegeros_interfaces', summary: '读取全部仿真接口' },
      { name: 'xuegeros_observe', summary: '读取状态、扇区与最近事件' },
      { name: 'xuegeros_propose', summary: '只预检，返回限幅距离与可用扇区' },
      { name: 'xuegeros_goal', summary: '直接下发并执行一个有界意图' },
      { name: 'xuegeros_stop', summary: '立即停车' },
      { name: 'xuegeros_look', summary: '单帧视觉问答' },
      { name: 'xuegeros_tour', summary: '房间巡游：开始/停止/读取原始测量' },
      { name: 'xuegeros_map', summary: '地图 PNG 链接与元信息' },
      { name: 'xuegeros_screenshot', summary: '仿真工作台截图或真车相机帧，带来源' },
      { name: 'xuegeros_ros', summary: '原样返回 ROS 分层与图信息' },
      { name: 'xuegeros_source', summary: '只读查看本仓库源码' },
    ],
    events: [
      {
        type: 'xuegeros.motion',
        description: {
          zh: '小脑否决/阻塞、命令未下发、命令格式错误，或巡游结束。普通 50 Hz 状态与常规完成不推事件，只记日志。',
          en: 'A veto/block, a command that did not dispatch, a malformed command, or a finished tour. Routine 50 Hz state and normal completion are not pushed.',
        },
        trigger: 'interrupt',
      },
      {
        type: 'xuegeros.fault',
        description: {
          zh: '心跳显示扫描或里程计越过新鲜度阈值（降级），或恢复。只在状态翻转时推一次。',
          en: 'The heartbeat shows scan/odom past their freshness limits (degraded), or recovered. Pushed once per transition.',
        },
        trigger: 'flush',
      },
    ],
  },
  create: (ctx) => new XuegerosWorld(ctx?.cfg, ctx),
};

export default XUEGEROS;
