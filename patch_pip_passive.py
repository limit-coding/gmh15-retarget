"""Patch GMH15 teleop: finger PIPs have no motor -> hide in UI and force 0 server-side.
Run inside ~/gmh15-sdk/teleop on the Pi."""


def sub(path, old, new):
    s = open(path, encoding="utf-8").read()
    assert s.count(old) == 1, (path, old[:60], s.count(old))
    open(path, "w", encoding="utf-8").write(s.replace(old, new))


NOTE = "7个电机关节可控：四指 MCP + 拇指 CMC/MCP/IP。四指 PIP/DIP 无独立电机，已隐藏并固定为0°。"

# --- server-side control
C = "teleop_control.py"
sub(C, "JOINT_NAMES = (",
    "# GMH15: 7 motors = 4 finger MCPs + thumb CMC/MCP/IP. Finger PIPs (11-index 1,3,5,7)\n"
    "# have no independent motor or feedback; SDK FK always reports 0. Commands must keep them at 0.\n"
    "PASSIVE = (1, 3, 5, 7)\nJOINT_NAMES = (")
sub(C, "def measured11(frame):",
    "def command11(values):\n"
    "    result = angles11(values)\n"
    "    for i in PASSIVE:\n"
    "        if result[i] != 0.0:\n"
    "            raise ValueError(f'{JOINT_NAMES[i]}无独立电机，目标必须为0°')\n"
    "    return result\n\n\n"
    "def measured11(frame):")
sub(C, "    return angles11([frame['joints_degrees'][i] for i in EDITABLE])",
    "    result = angles11([frame['joints_degrees'][i] for i in EDITABLE])\n"
    "    for i in PASSIVE:\n"
    "        result[i] = 0.0\n"
    "    return result")
sub(C, "        self.target = angles11(target)", "        self.target = command11(target)")

S = "teleop_server.py"
sub(S, "from teleop_control import angles11, EDITABLE, LIMITS, JOINT_NAMES",
    "from teleop_control import angles11, command11, EDITABLE, LIMITS, JOINT_NAMES, PASSIVE")
sub(S, "target=angles11(request.get('angles_degrees'))", "target=command11(request.get('angles_degrees'))")
sub(S, "request['angles_degrees']=angles11(request.get('angles_degrees'))",
    "request['angles_degrees']=command11(request.get('angles_degrees'))")
sub(S, "adjustable_indices_15=EDITABLE,", "adjustable_indices_15=EDITABLE,passive_indices=PASSIVE,")

# --- UI
P = "teleop_buttons.js"
sub(P, "function edit(h,index,value){",
    "const isPassive=i=>(config.passive_indices||[]).includes(i);\nfunction edit(h,index,value){if(isPassive(index))return;")
sub(P, "h.target=config.adjustable_indices_15.map(i=>actual.joints_degrees[i]);",
    "h.target=config.adjustable_indices_15.map((i,k)=>isPassive(k)?0:actual.joints_degrees[i]);")
sub(P, "const input=label.querySelector('input');h.inputs.push(input);",
    "const input=label.querySelector('input');if(isPassive(i)){label.hidden=true;label.style.display='none';input.disabled=true}h.inputs.push(input);")
sub(P, "if(nearest)h.selected=nearest.index;", "if(nearest&&!isPassive(nearest.index))h.selected=nearest.index;")
sub(P, "target:config.limits_degrees.map(([a])=>a),selected:0,", "target:config.limits_degrees.map(()=>0),selected:0,")
sub(P, "11个可编辑关节目标；其余四个 DIP 显示反馈角。目标模型不是实物反馈。", NOTE)
sub(P, "11个可编辑目标；四个 DIP 保留实测反馈角。目标模型不是实物反馈。", NOTE + "目标模型不是实物反馈。")
print("edits ok")
