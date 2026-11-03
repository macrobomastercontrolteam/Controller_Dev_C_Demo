const fs=require('node:fs');const path=require('node:path');
const root=__dirname;const profiles=JSON.parse(fs.readFileSync(path.join(root,'profiles.json'),'utf8'));
const rel='.archify/architecture-all-robots-20261004-212139';
const source=(path,line,end_line)=>({path,line,end_line});
function pair(id,file,tag,pos,line,end_line,headerLine=1,headerEnd=undefined){
  const h={path:'application/'+file+'.h',line:headerLine};if(headerEnd)h.end_line=headerEnd;
  return {id,type:['remote','cv'].includes(id)?'frontend':id==='can'?'messagebus':'backend',label:file+'.c',sublabel:file+'.h',tag,pos,size:[230,96],sources:[source('application/'+file+'.c',line,end_line),h]};
}
const optional=[
'Optional task pairs: vofa_task.c + vofa_task.h; oled_task.c + oled_task.h; usb_task.c + usb_task.h.',
'Optional task pairs: voltage_task.c + voltage_task.h; servo_task.c + servo_task.h; test_task.c + test_task.h.',
'These tasks are not created under the retained configuration; see Src/freertos.c:159-213.',
'graphic.c + graphic.h provides drawing helpers for custom_ui_task.c, whose drawing task is not created.'
];
for(const p of profiles){
  const s=p.type.toLowerCase(),folder=path.join(root,s);
  const runtime=JSON.parse(fs.readFileSync(path.join(folder,'candidate.json'),'utf8'));
  const original=path.join(folder,'candidate-before-files.json');if(!fs.existsSync(original))fs.copyFileSync(path.join(folder,'candidate.json'),original);
  const n=Object.fromEntries(runtime.components.map(n=>[n.id,n]));
  n.commands.label='Remote + CV modules';n.commands.sublabel='remote_control.c / .h';n.commands.tag='cv_usart_task.c / .h';
  n.ins.label='INS_task.c / .h';
  n.gimbal.label='gimbal_task.c / .h';n.gimbal.sublabel='gimbal_behaviour.c / .h';n.gimbal.tag='shoot.c / .h; 4 ms target';
  n.chassis.label='chassis_task.c / .h';n.chassis.sublabel='chassis_behaviour.c / .h';n.chassis.tag='5 ms; power '+(p.pwr?'ON':'OFF');
  n.referee.label=p.can?'referee_can_task.c / .h':'referee_usart_task.c / .h';
  n.referee.sublabel=p.can?'referee.c / .h; CAN state':'UART unpack off with VT13';
  runtime.cards=runtime.cards.filter(c=>!c.title.startsWith('Application files'));
  runtime.cards.push({dot:'cyan',title:'Application files: control + transport',items:[
    'application/gimbal_task.c + gimbal_task.h; gimbal_behaviour.c + gimbal_behaviour.h; shoot.c + shoot.h.',
    'application/chassis_task.c + chassis_task.h; chassis_behaviour.c + chassis_behaviour.h.',
    p.pwr?'application/chassis_power_control.c + chassis_power_control.h: active power allocation branch.':'application/chassis_power_control.c + chassis_power_control.h: power allocation branch disabled for this profile.',
    'application/CAN_receive.c + CAN_receive.h: shared motor TX/RX, referee bridge and supercap commands.',
    'application/remote_control.c + remote_control.h; cv_usart_task.c + cv_usart_task.h.',
    'application/'+(p.can?'referee_can_task':'referee_usart_task')+'.c + '+(p.can?'referee_can_task':'referee_usart_task')+'.h; referee.c + referee.h; protocol/protocol.h.'
  ]},{dot:'violet',title:'Application files: support + UI',items:[
    'application/INS_task.c + INS_task.h; detect_task.c + detect_task.h.',
    'application/calibrate_task.c + calibrate_task.h; led_flow_task.c + led_flow_task.h.',
    'application/custom_ui_task.c + custom_ui_task.h: ui_info state is used by active control code; drawing task is disabled.',
    ...optional
  ]});
  fs.writeFileSync(path.join(folder,'candidate.json'),JSON.stringify(runtime,null,2)+'\n');
  const d={schema_version:1,diagram_type:'architecture',meta:{
    title:p.type+' · Application files and selected dependencies',
    output:rel+'/'+s+'/application-files.html',locale:'en',quality_profile:'showcase',repository:runtime.meta.repository
  },components:[
    pair('remote','remote_control','Remote state / UART RX',[40,40],92,104,124,130),
    pair('ins','INS_task','IMU sampling / attitude',[400,40],171,274,78,118),
    pair('detect','detect_task','Health flags / offline checks',[760,40],76,195,120,143),
    pair('gimbal-behaviour','gimbal_behaviour','Mode / angle setpoints',[40,240],406,485,76,107),
    pair('gimbal','gimbal_task','4 ms target / motor safety',[400,240],203,322,33,34),
    pair('shoot','shoot','Launcher state / motor PID',[760,240],630,715,27,52),
    pair('cv','cv_usart_task',p.sentry?'CV UART / sentry timeouts':'CV UART / aim + move data',[40,440],171,262,1,35),
    pair('can','CAN_receive','CAN motor TX / RX + bridge',[400,440],1018,1150,28,84),
    pair('ui','custom_ui_task','ui_info active; drawing off',[760,440],19,34,24,68),
    pair('chassis-behaviour','chassis_behaviour','Motion mode / CV setpoints',[40,640],96,146,19,45),
    pair('chassis','chassis_task','5 ms target / '+p.wheel+' PID',[400,640],119,180,121,123),
    pair('power','chassis_power_control',p.pwr?'Power branch ON / Shift bypass':'Power branch OFF for this type',[760,640],550,585,288,294),
    pair('calibrate','calibrate_task','Flash / gyro + gimbal calibration',[40,840],423,477,182,202),
    pair('referee','referee','Match state / query API',[400,840],280,369,366,385),
    pair('ref-task',p.can?'referee_can_task':'referee_usart_task',p.can?'CAN UI TX task; 10 ms':'UART task; VT13 unpack off',[760,840],p.can?26:50,p.can?43:80,p.can?1:20,p.can?11:33),
    pair('led','led_flow_task','RGB task enabled',[40,1040],34,84,20,28),
    {id:'protocol',type:'backend',label:'protocol/protocol.h',sublabel:'Referee command IDs / packet structs',pos:[400,1040],size:[230,96],sources:[source('application/protocol/protocol.h',1,75),source('application/referee.c',1,11)]},
    pair('graphic','graphic','UI drawing helpers; task off',[760,1040],459,499,1,28)
  ],connections:[
    {id:'gimbal-modes',from:'gimbal',to:'gimbal-behaviour',label:'mode / setpoints'},
    {id:'read-rc',from:'gimbal-behaviour',to:'remote',label:'reads RC state'},
    {id:'read-ins',from:'gimbal',to:'ins',label:'reads attitude'},
    {id:'ins-health',from:'ins',to:'detect',label:'detect_hook'},
    {id:'shoot-loop',from:'gimbal',to:'shoot',label:'shoot_control_loop'},
    {id:'gimbal-can',from:'gimbal',to:'can',label:'motor TX',variant:'emphasis'},
    {id:'chassis-can',from:'chassis',to:'can',label:'drive + cap TX',variant:'emphasis'},
    {id:'chassis-modes',from:'chassis',to:'chassis-behaviour',label:'mode / setpoints'},
    {id:'chassis-cv',from:'chassis-behaviour',to:'cv',label:'reads CV motion'},
    {id:'can-ui',from:'can',to:'ui',label:p.can?'reads ui_info':'UI API; CAN bridge off',variant:p.can?'default':'dashed'},
    {id:'protocol-include',from:'referee',to:'protocol',label:'includes definitions'},

  ],cards:[
    {dot:'cyan',title:'Reading this application map',items:[
      'Every box names its application .c implementation and matching .h interface; click a box for source evidence.',
      'Arrows show selected caller-to-provider dependencies. State reads point toward the provider, not toward the data consumer.',
      'Both hardware and control-loop relationships remain in the companion system diagram; this map exposes application modules.',
      'All source links use branch revision d91be82e79a3d64802564e64a3184dfbeef54d5e.'
    ]},
    {dot:'emerald',title:'Robot configuration',items:runtime.cards.find(c=>c.title==='Robot-specific behavior').items},
    {dot:'amber',title:'Additional shared relationships',items:[
      'gimbal_task.c:748,1026 calls gimbal_behaviour APIs; :642 reads INS; :992-1013 reads CvCmdHandler aim state.',
      'chassis_task.c:177-180 reads remote, INS and gimbal feedback; :802 calls chassis_behaviour_control_set.',
      'shoot.c:689-715 uses referee state and CV modes. referee.c includes protocol/protocol.h and CAN_receive.h.',
      p.can?'referee_can_task.c:37 calls CAN_receive.c send_ui_info; CAN_receive.c:1577-1674 handles UI TX and referee RX.':'referee_usart_task.c has CRC/FIFO parsing for DR16 builds; VT13 disables that parse path under current globals.',
      'calibrate_task.c:532,574 applies gyro and gimbal calibration through their APIs and persists data in MCU flash.',
      'CAN_receive.c, INS_task.c and CV/remote receivers call detect_hook; control modules consume health flags.',
      'custom_ui_task.c:34 owns ui_info; gimbal and chassis modules update it; CAN reads it for bridge UI TX when enabled.'
    ]},
    {dot:'violet',title:'Optional application files',items:optional},
    {dot:'slate',title:'Scope',items:[
      'Nine non-biped configurations; all .c/.h labels are files in application/, except the nested protocol/protocol.h.',
      'CHASSIS_POWER_CONTROL, CAN_PASS_REF_INFO and special actuator paths vary by robot; global VT13/CV/MACRM settings are retained.',
      'Firmware edits are preserved and excluded from this committed snapshot; this is a source map, not a per-profile build verification.'
    ]}
  ]};
  if(p.pwr)d.connections.push({id:'chassis-power',from:'chassis',to:'power',label:'unless Shift bypass'});
  if(p.can){
    d.components.find(n=>n.id==='ref-task').sources.push(source('application/CAN_receive.c',1577,1674));
  }
  d.components.find(n=>n.id==='gimbal').sources.push(source('application/gimbal_task.c',738,748));
  d.components.find(n=>n.id==='ins').sources.push(source('application/INS_task.c',608,627));
  d.components.find(n=>n.id==='can').sources.push(source('application/CAN_receive.c',1577,1603));
  fs.writeFileSync(path.join(folder,'application-candidate.json'),JSON.stringify(d,null,2)+'\n');
}
console.log('Updated 9 runtime candidates; authored 9 application-file candidates.');


