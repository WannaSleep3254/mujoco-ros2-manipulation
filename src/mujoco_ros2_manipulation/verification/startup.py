"""Pure startup proofs shared by comparison production and verification."""
import re
from .process_faults import process_failures


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def controller_spawner_readiness(log, controllers):
    """Require three one-shot helpers to have exited before motion or startup cancellation cleanup.

    Activation does not prove that a Python spawner has finished its own
    teardown. The same strict proof is shared by production and verification.
    Cancellation and strict child-exit classification remain unchanged.
    """
    roles=("joint_state_broadcaster",*controllers)
    require(len(roles)==3 and len(set(roles))==3,"Expected three distinct controller startup helpers")
    # This fixed workcell launch emits simulation/state-publisher first, then
    # these helpers in role order. A changed launch must be reviewed explicitly.
    expected={"spawner-"+str(index):role for index,role in enumerate(roles,3)}
    plain=re.sub(r"\x1b\[[0-9;]*m","",log)
    lines=plain.splitlines()
    observed=dict(ready=False,reason="waiting_for_spawner_clean_exit",spawners=[])
    if process_failures(log) or any("process has died" in line for line in lines):
        observed["reason"]="child_process_failure"
        return observed
    if any("sending signal" in line or "user interrupted" in line for line in lines):
        observed["reason"]="launch_is_stopping"
        return observed
    starts=[];activated=[];exits=[]
    for number,line in enumerate(lines,1):
        match=re.fullmatch(r"\[INFO\] \[(spawner-[0-9]+)\]: process started with pid \[([0-9]+)\]",line)
        if match:starts.append((match[1],int(match[2]),number))
        match=re.fullmatch(r"\[(spawner-[0-9]+)\] \[INFO\] \[[^\]]+\] \[spawner_([^\]]+)\]: Configured and activated ([^ ]+)",line)
        if match:activated.append((match[1],match[2],match[3],number))
        match=re.fullmatch(r"\[INFO\] \[(spawner-[0-9]+)\]: process has finished cleanly \[pid ([0-9]+)\]",line)
        if match:exits.append((match[1],int(match[2]),number))
    names={item[0] for item in starts}
    all_names=set(re.findall(r"\[(spawner-[0-9]+)\]",plain))
    if (len(starts)!=3 or names!=set(expected) or len({item[1] for item in starts})!=3
            or any(item[1]<=1 for item in starts) or all_names!=names):
        observed["reason"]="missing_duplicate_or_unknown_spawner"
        return observed
    if (len(activated)!=3 or {item[0] for item in activated}!=names
            or {item[1] for item in activated}!=set(roles)
            or any(item[1]!=item[2] or item[1]!=expected[item[0]] for item in activated)):
        observed["reason"]="missing_duplicate_or_wrong_controller_activation"
        return observed
    if len(exits)!=3 or {item[0] for item in exits}!=names:
        return observed
    for name,pid,started in starts:
        activation=next(item for item in activated if item[0]==name)
        exited=next(item for item in exits if item[0]==name)
        if exited[1]!=pid or not started<activation[3]<exited[2]:
            observed["reason"]="wrong_spawner_pid_or_event_order"
            return observed
        observed["spawners"].append(dict(node=name,pid=pid,controller=activation[1],
            start_line=started,activated_line=activation[3],clean_exit_line=exited[2]))
    observed.update(ready=True,reason=None)
    return observed



def move_group_readiness(log, launch_pid, identify):
    """Bind final initialization to the single still-live owned MoveGroup."""
    lines = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", log).splitlines()
    starts = [(number, match[1], int(match[2])) for number, line in enumerate(lines, 1)
        if (match := re.fullmatch(r"\[INFO\] \[(move_group-[0-9]+)\]: process started with pid \[([0-9]+)\]", line))]
    if not starts:
        return dict(ready=False, reason='waiting_for_move_group_start')
    require(len(starts) == 1, 'Owned MoveGroup startup is ambiguous')
    started, name, pid = starts[0]
    require(not any(line.startswith(f'[INFO] [{name}]: process has finished cleanly') for line in lines),
            'Owned MoveGroup already exited before startup completed')
    identity = identify(pid)
    require(identity is not None and identity['pid'] == pid
            and identity['ppid'] == identity['pgid'] == identity['sid'] == launch_pid,
            'MoveGroup does not belong to this owned launch')
    contexts = [number for number, line in enumerate(lines, 1) if re.fullmatch(
        r'\[' + re.escape(name) + r'\] \[INFO\] \[[^\]]+\] '
        r'\[moveit_move_group_capabilities_base\.move_group_context\]: MoveGroup context initialization complete', line)]
    ready = [number for number, line in enumerate(lines, 1) if line == f'[{name}] You can start planning now!']
    require(len(contexts) <= 1 and len(ready) <= 1, 'Owned MoveGroup initialization markers are ambiguous')
    proof = dict(ready=False, reason='waiting_for_move_group_initialization', node=name, pid=pid,
                 start_ticks=identity['start_ticks'], start_line=started)
    if contexts and ready:
        require(started < contexts[0] < ready[0], 'Owned MoveGroup initialization markers are out of order')
        proof.update(ready=True, reason=None, context_ready_line=contexts[0], planning_ready_line=ready[0])
    return proof
