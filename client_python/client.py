import serial
import PySimpleGUI as sg
import threading
import sys
import time

COM_PORT_PICO_DEFAULT = "COM4"

joint_names = ['J1', 'J2', 'J3', 'J4', 'J5']

stop_threads = False
active_joint = ""

def thread_function(name):
    """A thread which only handles the incoming data from Pico and outputs it to console

    Args:
        name (string): name of thread (currently not used)
    """
    global stop_threads, ser
    while not stop_threads:
        while ser.in_waiting:
            data_in = ser.readline().decode("ascii")
            print("uC: " + data_in)

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
        

def send_start_cmd(joint, dir, value):
    """Send a start command to Pico for given joint and direction

    Args:
        joint (string): joint to be controlled (J1, J2, ...)
        dir (string): direction of joint movement (FORWARD, BACKWARD)
        value (number): speed of joint (0-255)
    """
    global active_joint
    active_joint = joint
    ser.write((joint + "_" + dir + "_START(" + str(int(value)) + ")\n").encode()) 

def send_set_pos_cmd(joint, value):
    """Send a set position command to Pico for given joint

    Args:
        joint (string): joint to be controlled (J1, J2, ...)
        value (number): position of joint
    """
    global active_joint
    active_joint = joint
    ser.write((joint + "_SET(" + str(float(value)) + ")\n").encode()) 

def send_end_cmd(dir):
    """Send a end command to Pico for the currently active joint

    Args:
        dir (string): direction of joint movement (FORWARD, BACKWARD)
    """
    global active_joint
    ser.write((active_joint + "_" + dir + "_END\n").encode())
    active_joint = ""

def send_init_cmd(joint):
    """Send init command to Pico for given joint

    Args:
        joint (string): joint to be controlled
    """
    ser.write((joint + "_INIT\n").encode())

def send_zero_cmd(joint):
    """Send zero command to Pico for given joint

    Args:
        joint (string): joint to be controlled
    """
    ser.write((joint + "_ZERO\n").encode())

if __name__ == "__main__":

    if len(sys.argv) == 2:
        com_port = sys.argv[1]
    else:
        com_port = COM_PORT_PICO_DEFAULT
    
    print("Used COM port: " + str(com_port))

    # open serial port
    ser = serial.Serial(com_port, 115200, timeout=0.05)

    # create all GUI elements
    chkbx_manual = sg.Checkbox("manual drive", enable_events = True, key="chkbx_manual")
    
    all_btns_manual = []
    for mo in joint_names:
        all_btns_manual.append(sg.Button("B", size=(4, 2)))
    for mo in joint_names:
        all_btns_manual.append(sg.Button("F", size=(4, 2)))
    for mo in joint_names:
        all_btns_manual.append(sg.Button("I", size=(4, 2)))
    for mo in joint_names:
        all_btns_manual.append(sg.Button("Z", size=(4, 2)))
    for mo in joint_names:
        all_btns_manual.append(sg.Button("S", size=(4, 2)))


    all_txts_manual = []
    for mo in joint_names:
        all_txts_manual.append(sg.Input(default_text="30", size=4))
    for mo in joint_names:
        all_txts_manual.append(sg.Input(default_text="0", size=4))

    btn_init = sg.Button("INIT", size=7)


    # create layouts
    button_layout = []
    for i in range(len(joint_names)):
        button_layout.append(sg.Column([[sg.Text(joint_names[i])], 
                                        [all_btns_manual[i]], 
                                        [all_btns_manual[i+len(joint_names)]], 
                                        [all_btns_manual[i+2*len(joint_names)]], 
                                        [all_btns_manual[i+3*len(joint_names)]], 
                                        [all_txts_manual[i]],
                                        [all_btns_manual[i+4*len(joint_names)]],
                                        [all_txts_manual[i+len(joint_names)]]
                                        ], element_justification='center'))

    txt_coord = sg.Input(default_text="0,0,0", size=20)
    btn_send_coord = sg.Button("SEND", size=7)
    
    layout = [  [sg.Text("Initialize all joints:")], 
                [btn_init],
                [sg.Text("Checkbox to enable the manual drive mode:")], 
                [chkbx_manual],
                [button_layout],
                [txt_coord],
                [btn_send_coord]
                ]

    # create the window
    window = sg.Window("StepperMotorControl", layout, margins=(50, 50), finalize=True)

    for i in range(len(all_btns_manual)):
        all_btns_manual[i].bind("<ButtonPress-1>", joint_names[i%len(joint_names)] + "_press")
    
    for col in button_layout:
        col.hide_row() 

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
            ser.write((txt_coord.get() + "\n").encode())
        
        elif event.endswith("press"):
            i = 0
            for mo in joint_names:
                if mo in event:
                    if event[0] == "I":
                        send_init_cmd(mo)
                    elif event[0] == "Z":
                        send_zero_cmd(mo)
                    elif event[0] == "S":
                        send_set_pos_cmd(mo, all_txts_manual[i+len(joint_names)].get())
                    else:
                        send_start_cmd(mo, dir, all_txts_manual[i].get())
                i = i + 1
        elif event[0] == "B" or event[0] == "F":
            send_end_cmd(dir)
        elif event == "chkbx_manual":
            if(chkbx_manual.get()):
                ser.write("ENABLE_MANUAL\n".encode())
                for col in button_layout:
                    col.unhide_row()
            else:
                ser.write("DISABLE_MANUAL\n".encode())
                for col in button_layout:
                    col.hide_row() 

        
    stop_threads = True
    window.close()
