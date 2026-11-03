const fs = require('node:fs');
const path = require('node:path');
const root = __dirname;
const base = JSON.parse(fs.readFileSync(path.join(root, 'base.json'), 'utf8'));
const profiles = [{"type":"INFANTRY_2023_MECANUM","line":87,"wheel":"Mecanum","motor":"3508","yaw":"GM6020","pitch":"GM6020","pwr":false,"can":false},{"type":"INFANTRY_2024_MECANUM","line":97,"wheel":"Mecanum","motor":"3508","yaw":"DM4310","pitch":"GM6020","pwr":true,"can":true},{"type":"INFANTRY_2023_SWERVE","line":107,"wheel":"Swerve","motor":"3508","yaw":"GM6020","pitch":"GM6020","pwr":false,"can":false,"special":"swerve"},{"type":"SENTRY_2023_MECANUM","line":127,"wheel":"Mecanum","motor":"3508","yaw":"GM6020","pitch":"GM6020","pwr":false,"can":false,"sentry":true},{"type":"HERO_2025_MECANUM","line":137,"wheel":"Mecanum","motor":"3508","yaw":"DM4310","pitch":"DM4340","pwr":true,"can":true,"special":"hero"},{"type":"SENTRY_2026_OMNI","line":147,"wheel":"Omni","motor":"4010","yaw":"DM4310","pitch":"DM3507","pwr":true,"can":true,"sentry":true},{"type":"INFANTRY_2026_MECANUM","line":157,"wheel":"Mecanum","motor":"3508","yaw":"DM4310","pitch":"DM4310","pwr":false,"can":true,"special":"hip"},{"type":"INFANTRY_2024_MECANUM_NEO","line":167,"wheel":"Mecanum","motor":"4010","yaw":"DM4310","pitch":"DM3507","pwr":true,"can":true,"active":true},{"type":"INFANTRY_2026_OMNI","line":177,"wheel":"Omni","motor":"4010","yaw":"DM4310","pitch":"DM3507","pwr":true,"can":true}];
const source = (path, line, end_line) => ({path,line,end_line});
for (const p of profiles) {
  const d = structuredClone(base);
  const slug = p.type.toLowerCase();
  const folder = path.join(root,slug);
  fs.mkdirSync(folder,{recursive:true});
  d.meta.title = p.type + ' · Firmware architecture';
  d.meta.output = '.archify/architecture-all-robots-20261004-212139/' + slug + '/' + slug + '.html';
  const n = Object.fromEntries(d.components.map(n => [n.id,n]));
  n.configuration.label = p.wheel + ' / ' + p.motor + ' profile';
  n.configuration.sublabel = 'Compile-time selection; power ' + (p.pwr ? 'ON' : 'OFF');
  n.configuration.sources = [source('Inc/global_inc.h',p.line,p.line+8)];
  n.commands.sublabel = p.sentry ? 'VT13 + CV; auto aim on startup' : 'VT13 remote + USART1 CV';
  n.gimbal.sublabel = '4 ms target; health / mode gates';
  n.gimbal.sources[0] = source('application/gimbal_task.c',203,322);
  n.gimbal.sources[1] = source('application/gimbal_task.h',33,34);
  n['gimbal-motors'].sublabel = p.yaw + ' yaw / ' + p.pitch + ' pitch';
  n['gimbal-motors'].sources[1] = source('application/CAN_receive.c',1018,1150);
  n.chassis.sublabel = '5 ms target; ' + p.wheel.toLowerCase() + '; power ' + (p.pwr ? 'ON' : 'OFF');
  n.chassis.sources[2] = source('application/chassis_task.h',121,123);
  n.referee.label = p.can ? 'Referee via CAN bridge' : 'Local referee UART task';
  n.referee.sublabel = p.can ? 'CAN RX state + UI TX task' : 'Unpack disabled with current VT13';
  n.drive.label = 'Four ' + p.motor + ' drive motors';
  n.drive.sublabel = p.motor === '4010' ? 'CAN1 / KTech broadcast speed' : 'CAN1 / DJI current commands';
  n.drive.sources.push(source('application/CAN_receive.c',p.motor === '4010' ? 1350 : 1299,p.motor === '4010' ? 1394 : 1346));
  n.supercap.label = 'MACRM supercapacitor';
  n.supercap.sublabel = 'CAN command; selected global variant';
  n.supercap.sources[0] = source('Inc/global_inc.h',38,50);
  d.connections.find(e => e.id==='gimbal-can').label = 'CAN1 + CAN2';
  d.connections.find(e => e.id==='chassis-can').label = p.motor==='4010' ? 'speed broadcast' : 'current command';
  d.connections.find(e => e.id==='cap-command').label = 'MACRM CAN';
  d.connections.find(e => e.id==='ref-chassis').label = p.can ? 'power / buffer' : 'state if UART active';
  if (!p.can) d.connections.find(e => e.id==='ref-chassis').variant = 'dashed';
  const details = [
    'Hardware map: Inc/global_inc.h:' + p.line + '-' + (p.line+8) + '.',
    p.active ? 'This is the selected ROBOT_TYPE in the pinned commit.' : 'This profile is shown with ROBOT_TYPE set to ' + p.type + '; other global options retained.',
    'Yaw ' + p.yaw + '; pitch ' + p.pitch + '; drive ' + p.motor + '; ' + p.wheel + ' kinematics.',
    p.pwr ? 'Active chassis power control; Shift with buffer > 30 bypasses it (chassis_task.c:1195-1212).' : 'CHASSIS_POWER_CONTROL=0: no active firmware power allocation loop.',
    p.can ? 'CAN_PASS_REF_INFO=1: CAN RX parses referee state; referee_can_task sends UI information.' : 'CAN_PASS_REF_INFO=0: UART task selected, but REMOTE_TYPE=VT13 disables its referee FIFO unpacking.'
  ];
  if (p.sentry) details.push('CV auto aim enabled at startup; shoot, spin and align modes have timeouts (cv_usart_task.c:177-207).');
  if (p.special==='swerve' || p.special==='hip') {
    d.components.push({
      id:'special-chassis',type:'external',
      label:p.special==='swerve' ? 'Steering + hip controllers' : 'Chassis hip controller',
      sublabel:p.special==='swerve' ? 'Steering encoders; hip geometry' : 'Angle / height / hip gain',
      pos:[860,660],size:[230,76],
      sources:[source('application/CAN_receive.c',p.special==='swerve' ? 1398 : 1463,p.special==='swerve' ? 1456 : 1491)]
    });
    d.connections.push({id:'special-chassis-can',from:'chassis',to:'special-chassis',label:'CAN1; enable gates'});
    details.push(p.special==='swerve' ? 'Steer output requires ENABLE_STEER; hip geometry requires ENABLE_HIP and fHipEnabled. Disabled packets use 0xFF bytes.' : 'Hip packet requires ENABLE_HIP and fHipEnabled; disabled packets use 0xFF bytes.');
  }
  if (p.special==='hip') {
    d.components.push({id:'pitch-base',type:'external',label:'DM4310 pitch-base motor',sublabel:'Additional MIT position / torque path',pos:[860,40],size:[230,76],sources:[source('application/CAN_receive.c',686,705),source('application/gimbal_task.c',270,272)]});
    d.connections.push({id:'pitch-base-can',from:'gimbal',to:'pitch-base',label:'CAN2; motor enabled'});
    details.push('Pitch and pitch-base outputs use CAN_cmd_gimbal_Damiao_motor; startup also waits for the pitch-base motor.');
  }
  if (p.special==='hero') {
    d.components.push({id:'hero-launcher',type:'external',label:'Hero extra launcher motors',sublabel:'Friction 3 / 4 + piston; gated OFF',pos:[860,40],size:[230,76],sources:[source('application/shoot.c',654,664),source('application/CAN_receive.c',1042,1044),source('application/CAN_receive.c',1120,1126)]});
    d.connections.push({id:'hero-launcher-can',from:'gimbal',to:'hero-launcher',label:'CAN2; switch gates',variant:'dashed'});
    details.push('Hero controls four friction motors and a piston; ENABLE_SHOOT_REDUNDANT_SWITCH=0 forces extra friction and piston command outputs to zero.');
  }
  if (p.type==='SENTRY_2023_MECANUM') details.push('Upper-head CAN sender exists but has no call site in this branch; it is not shown as an active runtime path.');
  d.cards = [
    {dot:'emerald',title:'Robot-specific behavior',items:details},
    {dot:'cyan',title:'Control and feedback',items:[
      'FreeRTOS: INS is realtime; gimbal high; chassis above normal (Src/freertos.c:164-177).',
      'Gimbal + launcher target 4 ms; chassis target 5 ms. Blocking CAN delays can affect elapsed loop timing.',
      'Motor CAN RX updates feedback; chassis also reads INS attitude and gimbal encoder state (chassis_task.c:177-180).',
      'Gimbal waits for required motors at startup; relevant offline motors cause zero output (gimbal_task.c:203-322).',
      'Referee heat and shooter-enable state affect launcher behavior (CAN_receive.c:1680-1683; shoot.c:689-715).'
    ]},
    {dot:'amber',title:'Retained global configuration',items:[
      'REMOTE_TYPE=VT13; CV_INTERFACE=1; DEBUG_CV=1; SUPERCAP_TYPE=MACRM; COMPETITION_TYPE=RMUL.',
      'ENABLE_SHOOT_REDUNDANT_SWITCH=0 selects low normal friction speed; it is not a blanket launcher-disable switch.',
      'Actuator output also depends on enable flags in CAN_receive.c:44-102.',
      'Diagrams describe committed source paths, not measured robot behavior or verified builds of every profile.'
    ]},
    {dot:'violet',title:'Supporting firmware',items:[
      'Detection monitors device health. Calibration reads/writes MCU flash (calibrate_task.c:423-477). RGB task is enabled.',
      'VOFA is disabled in current globals. OLED, custom UI, USB, battery and servo task creation is commented out (Src/freertos.c:179-210).'
    ]},
    {dot:'slate',title:'Source snapshot',items:[
      'Branch Infantry_Common_Code_Opt; revision d91be82e79a3d64802564e64a3184dfbeef54d5e.',
      'Biped excluded; existing local firmware edits preserved and excluded from source evidence.'
    ]}
  ];
  fs.writeFileSync(path.join(folder,'candidate.json'),JSON.stringify(d,null,2)+'\n');
}
fs.writeFileSync(path.join(root,'profiles.json'),JSON.stringify(profiles,null,2)+'\n');

