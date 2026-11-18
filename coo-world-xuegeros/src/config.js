export const ID = 'xuegeros';
export const LABEL = 'XuegerOS 小车';

export const DEFAULTS = {
  // Disabled by default: the operator enables it in Coo.
  enabled: false,
  // New installs open in simulation. Saved physical deployments are explicit.
  deployment: 'simulation',
  // robotd's WebSocket, reached through ssh -L. No network port on the car.
  robotdUrl: 'ws://127.0.0.1:18202/ws',
  // Deliberately separate loopback port for the local simulator. Keeping this
  // distinct from the physical tunnel makes an accidental cross-connection
  // visible in configuration and easy to reject.
  simulationRobotdUrl: 'ws://127.0.0.1:18242/ws',
  simulationHttpUrl: 'http://127.0.0.1:18142',
  // World-specific restricted token; robotd rejects a missing/wrong token.
  robotdToken: '',
  // Optional legacy console URL for status consumers.
  localConsoleUrl: 'http://127.0.0.1:18112',
  // Read-only camera frame + VLM for xuegeros_look.
  frameUrl: 'http://127.0.0.1:18110/v1/frames/latest.jpg',
  vlmUrl: 'http://127.0.0.1:18083',
  // Read-only artifacts and raw ROS facts, all served by the body adapter.
  mapUrl: 'http://127.0.0.1:18110/v1/map.png',
  mapMetaUrl: 'http://127.0.0.1:18110/v1/map',
  stateUrl: 'http://127.0.0.1:18110/v1/state',
  graphUrl: 'http://127.0.0.1:18110/v1/graph',
  // Empty means "derive from this extension's own folder"; override only when
  // the repository lives elsewhere.
  sourceRoot: '',
  lookTimeoutMs: 60000,
  requestTimeoutMs: 4000,
  // Persisted event log; relative paths resolve against ctx.dataDir.
  eventLogPath: 'xuegeros-events.jsonl',
  maxReplay: 200,
};

export const WAKE_KINDS = new Set([
  'motion_vetoed',
  'motion_blocked',
  'command_failed',
  'command_error',
  'tour_done',
  // Older logs used these names; replay still has to wake on them.
  'proposal_refused',
  'execute_refused',
]);

export const WAKE_STATES = new Set(['VETO', 'BLOCKED']);

export const CONFIG_GROUP = {
  id: 'world:xuegeros',
  owner: 'world:xuegeros',
  schema: {
    type: 'object',
    title: 'XuegerOS 小车',
    description: '经 robotd 协商动作：没有人工计时授权，主控用扫描几何限幅并可否决。',
    properties: {
      'worlds.xuegeros.deployment': {
        type: 'string',
        enum: ['physical', 'simulation'],
        title: '运行目标',
        description: 'simulation 使用单独配置的仿真地址；physical 使用车端 robotd 隧道。',
        'x-hot': true,
      },
      'worlds.xuegeros.robotdUrl': {
        type: 'string',
        title: 'robotd WebSocket',
        description: '通常经 ssh -L 转发到 robotd 的 Unix socket，例如 ws://127.0.0.1:18202/ws。',
        'x-hot': true,
      },
      'worlds.xuegeros.simulationRobotdUrl': {
        type: 'string',
        title: '仿真 robotd WebSocket',
        description: '仅 deployment=simulation 使用，可填写本机或局域网地址。',
        'x-hot': true,
      },
      'worlds.xuegeros.robotdToken': {
        type: 'string',
        title: 'robotd token',
        description: 'World 专用受限 token；留空表示 robotd 未启用 token（不推荐）。',
        'x-hot': true,
      },
      'worlds.xuegeros.localConsoleUrl': {
        type: 'string',
        title: '本地中控地址',
        description: '独立真车中控 tools/robot_console.py 的兼容地址。',
        'x-hot': true,
      },
      'worlds.xuegeros.frameUrl': {
        type: 'string',
        title: '相机帧地址',
        description: 'Body Adapter 的最新帧，只给 xuegeros_look 用。',
        'x-hot': true,
      },
      'worlds.xuegeros.vlmUrl': {
        type: 'string',
        title: '视觉模型地址',
        description: 'llama-server 根地址。',
        'x-hot': true,
      },
      'worlds.xuegeros.eventLogPath': {
        type: 'string',
        title: '事件日志文件',
        description: '相对路径按 World 的 dataDir 解析；用于重启后补投被拒/否决事件。',
        'x-hot': false,
      },
    },
  },
};

// Every supported setting is editable through the host configuration panel.
const TITLES = {enabled:'启用', simulationHttpUrl:'仿真工作台地址', mapUrl:'地图图片地址',
  mapMetaUrl:'地图元数据地址', stateUrl:'ROS 状态地址', graphUrl:'ROS 图地址', sourceRoot:'源码目录',
  lookTimeoutMs:'视觉超时 · ms', requestTimeoutMs:'请求超时 · ms', maxReplay:'事件回放上限'};
for (const [key, value] of Object.entries(DEFAULTS)) {
  const name = `worlds.xuegeros.${key}`;
  if (!CONFIG_GROUP.schema.properties[name]) CONFIG_GROUP.schema.properties[name] = {
    type: typeof value === 'number' ? 'integer' : typeof value,
    title: TITLES[key] || key, 'x-hot': !['eventLogPath','sourceRoot'].includes(key),
    ...(typeof value === 'number' ? {minimum:1} : {}),
  };
}
