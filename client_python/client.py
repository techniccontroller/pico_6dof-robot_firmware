import serial
import PySimpleGUI as sg
import threading
import sys
import time
import json

COM_PORT_PICO_DEFAULT = "COM5"

joint_names = ['J1', 'J2', 'J3', 'J4', 'J5']
motor_names = ['M1', 'M2', 'M3', 'M4', 'M5']

stop_threads = False
active_joint_motor = ""

robot_data = dict()
all_lbls_joints = []

def thread_function(name):
    """A thread which only handles the incoming data from Pico and outputs it to console

    Args:
        name (string): name of thread (currently not used)
    """
    global stop_threads, ser, robot_data, all_lbls_joints
    while not stop_threads:
        while ser.in_waiting:
            try:
                data_in = ser.readline().decode("ascii")
                print("uC: " + data_in)

                if data_in.strip().startswith("{") and data_in.strip().endswith("}"):
                    try:
                        json_data = json.loads(data_in)
                        if "robot_data" in json_data:
                            robot_data = json_data["robot_data"]
                            update_lbls(all_lbls_joints)
                    except json.JSONDecodeError:
                        print("Error decoding JSON data")
            except UnicodeDecodeError:
                print("Error decoding incoming data")

def thread_square(name):

    global ser

    x_range = [0.0, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.0]
    y_range = [0.2, 0.2, 0.2, 0.3, 0.3, 0.2,  0.2,  0.2,  0.3,  0.3,  0.2, 0.2]
    z_range = [0.1, 0.1, 0.2, 0.2, 0.1, 0.1,  0.1,  0.2,  0.2,  0.1,  0.1, 0.1]

    #x_range = [   0.0,    0.1,    0.0,   -0.1,    0.0]
    #y_range = [   0.2,    0.2,    0.2,    0.2,    0.2]
    #z_range = [-0.117, -0.117, -0.117, -0.117, -0.117]

    x = x_range[0]
    y = y_range[0]
    z = z_range[0]
    for i in range(len(x_range)):

        x_delta = (x_range[i] - x)/40
        y_delta = (y_range[i] - y)/40
        z_delta = (z_range[i] - z)/40

        for i in range(40):
            x_tmp = x + i*x_delta
            y_tmp = y + i*y_delta
            z_tmp = z + i*z_delta
            ser.write(("COORD(" + str("%.3f" % x_tmp) + "," + str("%.3f" % y_tmp) + "," + str("%.3f" % z_tmp) + ")\n").encode())
            time.sleep(0.05)

        x = x_tmp
        y = y_tmp
        z = z_tmp
        

def send_start_cmd(joint_motor, dir, value):
    """Send a start command to Pico for given joint or motor

    Args:
        joint_motor (string): joint or motor to be controlled (J1, J2, M1, M2, ...)
        dir (string): direction of movement (FORWARD, BACKWARD)
        value (number): speed (0-255)
    """
    global active_joint_motor
    active_joint_motor = joint_motor
    ser.write(("CMD_" + joint_motor + "_" + dir + "_START(" + str(int(value)) + ")\n").encode()) 

def send_set_pos_cmd(joint_motor, value):
    """Send a set position command to Pico for given joint or motor

    Args:
        joint_motor (string): joint or motor to be controlled (J1, J2, M1, M2, ...)
        value (number): position
    """
    global active_joint_motor
    active_joint_motor = joint_motor
    ser.write(("CMD_" + joint_motor + "_SET(" + str(float(value)) + ")\n").encode()) 

def send_end_cmd(dir):
    """Send a end command to Pico for the currently active joint or motor

    Args:
        dir (string): direction of movement (FORWARD, BACKWARD)
    """
    global active_joint_motor
    ser.write(("CMD_" + active_joint_motor + "_" + dir + "_END\n").encode())
    active_joint_motor = ""

def send_init_cmd(joint_motor):
    """Send init command to Pico for given joint or motor

    Args:
        joint_motor (string): joint to be controlled
    """
    ser.write(("CMD_" + joint_motor + "_INIT\n").encode())

def send_zero_cmd(joint_motor):
    """Send zero command to Pico for given joint or motor

    Args:
        joint_motor (string): joint or motor to be controlled
    """
    ser.write(("CMD_" + joint_motor + "_ZERO\n").encode())

def create_btns_joint(names):
    
    all_btns_manual = []
    for mo in names:
        all_btns_manual.append(sg.Button("B", size=(4, 2)))
    for mo in names:
        all_btns_manual.append(sg.Button("F", size=(4, 2)))
    for mo in names:
        all_btns_manual.append(sg.Button("Z", size=(4, 2)))
    for mo in names:
        all_btns_manual.append(sg.Button("S", size=(4, 2)))
    return all_btns_manual

def create_btns_motor(names):
    
    all_btns_manual = []
    for mo in names:
        all_btns_manual.append(sg.Button("B", size=(4, 2)))
    for mo in names:
        all_btns_manual.append(sg.Button("F", size=(4, 2)))
    return all_btns_manual

def create_txts_joint(names):    
    all_txts_manual = []
    for mo in names:
        if mo == "J1" or mo == "J2" or mo == "J3":
            all_txts_manual.append(sg.Input(default_text="10", size=4))
        else:
            all_txts_manual.append(sg.Input(default_text="50", size=4))
    for mo in names:
        all_txts_manual.append(sg.Input(default_text="0", size=4))
    return all_txts_manual

def create_lbls_joint(names):
    all_lbls_manual = []
    for mo in names:
        all_lbls_manual.append(sg.Text("-", size=4))
    return all_lbls_manual

def create_txts_motor(names):    
    all_txts_manual = []
    for mo in names:
        if mo == "M1" or mo == "M2" or mo == "M3":
            all_txts_manual.append(sg.Input(default_text="10", size=4))
        else:
            all_txts_manual.append(sg.Input(default_text="50", size=4))
    return all_txts_manual

def create_btn_layout_joint(names, btns, txts, lbls):
    button_layout = []
    for i in range(len(names)):
        button_layout.append(sg.Column([[sg.Text(names[i])], 
                                        [btns[i]], 
                                        [btns[i+len(names)]], 
                                        [btns[i+2*len(names)]],
                                        [sg.Text("Vel:")], 
                                        [txts[i]],
                                        [btns[i+3*len(names)]],
                                        [sg.Text("Set pos:")],  
                                        [txts[i+len(names)]],
                                        [sg.Text("Current pos:")],
                                        [lbls[i]]
                                        ], element_justification='center'))
    return button_layout

def create_btn_layout_motor(names, btns, txts):
    button_layout = []
    for i in range(len(names)):
        button_layout.append(sg.Column([[sg.Text(names[i])], 
                                        [btns[i]], 
                                        [btns[i+len(names)]],
                                        [sg.Text("Vel:")], 
                                        [txts[i]]
                                        ], element_justification='center'))
    return button_layout

def update_lbls(lbls):
    global robot_data
    for i in range(len(lbls)):
        if "config" in robot_data:
            lbls[i].update(int(robot_data["config"][i]))


if __name__ == "__main__":

    if len(sys.argv) == 2:
        com_port = sys.argv[1]
    else:
        com_port = COM_PORT_PICO_DEFAULT
    
    print("Used COM port: " + str(com_port))

    # open serial port
    ser = serial.Serial(com_port, 115200, timeout=0.05)

    # radio button group with 3 radio buttons (AUTO, JOINT, MOTOR)
    radio_layout_mode = [[sg.Radio("AUTO", "RADIO1", default=True, key="AUTO", enable_events=True), sg.Radio("JOINT", "RADIO1", key="JOINT", enable_events=True), sg.Radio("MOTOR", "RADIO1", key="MOTOR", enable_events=True)]]

    # create buttons and text fields
    all_btns_joints = create_btns_joint(joint_names)
    all_btns_motors = create_btns_motor(motor_names)
    all_txts_joints = create_txts_joint(joint_names)
    all_txts_motors = create_txts_motor(motor_names)
    all_lbls_joints = create_lbls_joint(joint_names)

    btn_init = sg.Button("INIT", size=7)

    # create layouts
    button_layout_joints = create_btn_layout_joint(joint_names, all_btns_joints, all_txts_joints, all_lbls_joints)
    button_layout_motors = create_btn_layout_motor(motor_names, all_btns_motors, all_txts_motors)
    
    

    txt_custom = sg.Input(default_text="0,0,0", size=20)
    btn_send_coord = sg.Button("SEND", size=7)
    btn_save_zeros = sg.Button("SAVE ZEROS", size=15)
    btn_load_zeros = sg.Button("LOAD ZEROS", size=15)
    btn_gripper_open = sg.Button("OPEN", size=15)
    btn_gripper_close = sg.Button("CLOSE", size=15)
    btn_gripper_set = sg.Button("SET", size=15)
    txt_gripper = sg.Input(default_text="0", size=4)

    container_btn_layout_joints = sg.Column([   [sg.HorizontalSeparator()],
                                                [sg.Text("Joint control:", size=(60, 2), justification='center')], 
                                                button_layout_joints,
                                                [sg.HorizontalSeparator()]], element_justification='center')
    
    container_btn_layout_motors = sg.Column([   [sg.HorizontalSeparator()],
                                                [sg.Text("Motor control:", size=(60, 2), justification='center')], 
                                                button_layout_motors,
                                                [sg.HorizontalSeparator()]], element_justification='center')

    container_gripper_layout = sg.Column([      [sg.HorizontalSeparator()],
                                                [sg.Text("Gripper control:", size=(60, 2), justification='center')], 
                                                [btn_gripper_open, btn_gripper_close],
                                                [sg.Text("Set position:"), txt_gripper, btn_gripper_set],
                                                [sg.HorizontalSeparator()]], element_justification='center')

    
    layout = [  [sg.Text("Initialize all joints:")], 
                [btn_init],
                [sg.Text("Select the control mode:")], 
                [radio_layout_mode],
                [container_btn_layout_joints],
                [container_btn_layout_motors],
                [txt_custom],
                [btn_send_coord],
                [btn_save_zeros],
                [btn_load_zeros],
                [container_gripper_layout],
                ]

    # create the window
    window = sg.Window("StepperMotorControl", layout, margins=(50, 50), finalize=True)
    
    container_btn_layout_joints.hide_row()
    container_btn_layout_motors.hide_row()

    for i in range(len(all_btns_joints)):
        all_btns_joints[i].bind("<ButtonPress-1>", joint_names[i%len(joint_names)] + "_press")
    
    for i in range(len(all_btns_motors)):
        all_btns_motors[i].bind("<ButtonPress-1>", motor_names[i%len(motor_names)] + "_press")

    # start thread for handling serial inputs
    x = threading.Thread(target=thread_function, args=(1,), daemon=True)
    x.start()

    # create an event loop
    while True:
        event, values = window.read()
        # end program if user closes window
        if event == sg.WIN_CLOSED:
            break
        
        # filter receiving events from window and act accordently
        event = str(event)
        print(event)
        if event[0] == "B": 
            dir = "BACKWARD"
        else: 
            dir = "FORWARD"
        
        if event == "INIT":
            x = threading.Thread(target=thread_square, args=(1,), daemon=True)
            x.start()
        
        elif event == "SEND":
            #ser.write(("COORD(" + txt_coord.get() + ")\n").encode())
            ser.write((txt_custom.get() + "\n").encode())
        
        elif event == "SAVE ZEROS":
            ser.write(("SAVE_ZEROS\n").encode())

        elif event == "LOAD ZEROS":
            ser.write(("LOAD_ZEROS\n").encode())
        
        elif event == "OPEN":
            ser.write(("GRIP_OPEN\n").encode())

        elif event == "CLOSE":
            ser.write(("GRIP_CLOSE\n").encode())
        
        elif event == "SET":
            ser.write(("GRIP_SET(" + txt_gripper.get() + ")\n").encode())
        
        elif event.endswith("press"):
            i = 0
            for jo in joint_names:
                if jo in event:
                    if event[0] == "Z":
                        send_zero_cmd(jo)
                    elif event[0] == "S":
                        send_set_pos_cmd(jo, all_txts_joints[i+len(joint_names)].get())
                    else:
                        send_start_cmd(jo, dir, all_txts_joints[i].get())
                i = i + 1
            i = 0
            for mo in motor_names:
                if mo in event:
                        send_start_cmd(mo, dir, all_txts_motors[i].get())
                i = i + 1
        elif event[0] == "B" or event[0] == "F":
            send_end_cmd(dir)
        elif event == "AUTO":
            ser.write("SET_MODE_AUTO\n".encode())
            container_btn_layout_joints.hide_row()
            container_btn_layout_motors.hide_row()
        elif event == "JOINT":
            ser.write("SET_MODE_JOINT\n".encode())
            container_btn_layout_joints.unhide_row()
            container_btn_layout_motors.hide_row()
        elif event == "MOTOR":
            ser.write("SET_MODE_MOTOR\n".encode())
            container_btn_layout_joints.hide_row()
            container_btn_layout_motors.unhide_row()
        

        
    stop_threads = True
    window.close()
