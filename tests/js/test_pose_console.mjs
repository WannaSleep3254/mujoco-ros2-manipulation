// Execute the real pure goal helpers; no browser, API or renderer is started.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const source=readFileSync(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/app.js',import.meta.url),'utf8');
const taskHelpers=source.slice(source.indexOf('function graspSelected('),source.indexOf('let config,'));
const helperSource=source.slice(source.indexOf('function goalKind()'),source.indexOf('function strategyLabel('));
assert.ok(helperSource.includes('function goalMatchesPayload('),'Pose input helpers must be present');
function fixture(){
 const elements=Object.fromEntries(Object.entries({'goal-kind':'pose','pose-arm':'left','pose-x':'0.1','pose-y':'0.2','pose-z':'0.3','pose-qx':'0','pose-qy':'0','pose-qz':'0','pose-qw':'1','strategy':'coordinated'}).map(([id,value])=>[id,{value}]));
 const context={config:{pose_tools:[{arm_id:'left',tool:{id:'left_flange_reference',parent_tip:'left_tip',sha256:'c'.repeat(64)}}]},targets:{l:.1,r:0},state:null,taskControls:null,taskCommandBusy:false,$:id=>elements[id]||null,validTargets:()=>true};
 vm.createContext(context);vm.runInContext(taskHelpers+helperSource,context);
 return {context,elements};
}
let checks=0;
function check(name,operation){operation();checks++;console.log(`ok ${checks} - ${name}`);}
check('Pose input carries explicit world, meters, registered arm and tool digest',()=>{
 const {context}=fixture(),goal=JSON.parse(JSON.stringify(context.poseGoalPayload('b'.repeat(32))));
 assert.deepEqual(goal,{schema_version:1,request_id:'b'.repeat(32),arm_id:'left',tool_id:'left_flange_reference',tool_sha256:'c'.repeat(64),frame_id:'world',position_m:[.1,.2,.3],quaternion_xyzw:[0,0,0,1]});
});
check('near-unit quaternion uses the same normalization as server validation',()=>{
 const {context,elements}=fixture();elements['pose-qw'].value='1.0000005';
 const goal=context.poseGoalPayload('b'.repeat(32));assert.equal(goal.quaternion_xyzw[3],1);
 assert.equal(context.goalMatchesPayload(goal),true);
});
check('blank and nonfinite positions are never silently converted to zero',()=>{
 for(const value of ['',' ','NaN','Infinity','1e309']){const {context,elements}=fixture();elements['pose-x'].value=value;assert.throws(()=>context.poseGoalPayload('b'.repeat(32)));assert.equal(context.validGoal(),false);}
});
check('non-unit quaternion is refused before a plan can be requested',()=>{
 const {context,elements}=fixture();elements['pose-qw'].value='90';assert.throws(()=>context.poseGoalPayload('b'.repeat(32)),/quaternion/);assert.equal(context.validGoal(),false);
});
check('unknown arm has no inferred TCP fallback',()=>{
 const {context,elements}=fixture();elements['pose-arm'].value='right';assert.throws(()=>context.poseGoalPayload('b'.repeat(32)));assert.equal(context.validGoal(),false);
});
check('changing position, arm or tool digest invalidates review input match',()=>{
 for(const change of ['position','arm','tool']){const {context,elements}=fixture();const goal=context.poseGoalPayload('b'.repeat(32));if(change==='position')elements['pose-x'].value='.11';else if(change==='arm')elements['pose-arm'].value='right';else context.config.pose_tools[0].tool.sha256='d'.repeat(64);assert.equal(context.goalMatchesPayload(goal),false);}
});
check('switching goal kind cannot approve the other kind of reviewed input',()=>{
 const {context,elements}=fixture(),pose=context.poseGoalPayload('b'.repeat(32));
 const joint={schema_version:1,request_id:'e'.repeat(32),strategy:'coordinated',target_rad:{l:.1,r:0}};
 assert.equal(context.goalMatchesPayload(joint),false);elements['goal-kind'].value='joint';assert.equal(context.goalMatchesPayload(pose),false);assert.equal(context.goalMatchesPayload(joint),true);
});
check('Pose planning requires the dedicated server capability',()=>{
 const {context,elements}=fixture();assert.equal(context.canPlanGoal({can_plan:true,can_pose_plan:false}),false);assert.equal(context.canPlanGoal({can_plan:false,can_pose_plan:true}),true);elements['goal-kind'].value='joint';assert.equal(context.canPlanGoal({can_plan:true,can_pose_plan:false}),true);
});
check('native task selection and uncertain task changes block both normal planning modes',()=>{
 const {context,elements}=fixture();for(const kind of ['joint','pose']){elements['goal-kind'].value=kind;assert.equal(context.canPlanGoal({can_plan:true,can_pose_plan:true,task:{selected:'grasp'}}),false);context.taskCommandBusy=true;assert.equal(context.canPlanGoal({can_plan:true,can_pose_plan:true}),false);context.taskCommandBusy=false;context.taskControls={unconfirmed:{action:'select'}};assert.equal(context.canPlanGoal({can_plan:true,can_pose_plan:true}),false);context.taskControls=null;}
});
console.log(`${checks} Pose console input scenarios passed`);
