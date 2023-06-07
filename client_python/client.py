import serial
import PySimpleGUI as sg
import threading
import sys

COM_PORT_PICO_DEFAULT = "COM4"

motor_names = ['M1', 'M2', 'M3', 'M4']

stop_threads = False
active_motor = ""

def thread_function(name):
    """A thread which only handles the incoming data from Pico and outputs it to console

    Args:
        name (string): name of thread (currently not used)
    """
    global stop_threads
    while not stop_threads:
        while ser.in_waiting:
            data_in = ser.readline().decode("ascii")
            print("uC: " + data_in)

def send_start_cmd(motor, dir, value):
    """Send a start command to Pico for given motor and direction

    Args:
        motor (string): motor to be controlled (M1, M2, ...)
        dir (string): direction of motor movement (FORWARD, BACKWARD)
        value (number): speed of motor (0-255)
    """
    global active_motor
    active_motor = motor
    ser.write((motor + "_" + dir + "_START(" + str(int(value)) + ")\n").encode()) 

def send_set_pos_cmd(motor, value):
    """Send a set position command to Pico for given motor

    Args:
        motor (string): motor to be controlled (M1, M2, ...)
        value (number): position of motor
    """
    global active_motor
    active_motor = motor
    ser.write((motor + "_SET(" + str(float(value)) + ")\n").encode()) 

def send_end_cmd(dir):
    """Send a end command to Pico for the currently active motor

    Args:
        dir (string): direction of motor movement (FORWARD, BACKWARD)
    """
    global active_motor
    ser.write((active_motor + "_" + dir + "_END\n").encode())
    active_motor = ""

def send_init_cmd(motor):
    """Send init command to Pico for given motor

    Args:
        motor (string): motor to be controlled
    """
    ser.write((motor + "_INIT\n").encode())

def send_zero_cmd(motor):
    """Send zero command to Pico for given motor

    Args:
        motor (string): motor to be controlled
    """
    ser.write((motor + "_ZERO\n").encode())

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
    for mo in motor_names:
        all_btns_manual.append(sg.Button("B", size=(4, 2)))
    for mo in motor_names:
        all_btns_manual.append(sg.Button("F", size=(4, 2)))
    for mo in motor_names:
        all_btns_manual.append(sg.Button("I", size=(4, 2)))
    for mo in motor_names:
        all_btns_manual.append(sg.Button("Z", size=(4, 2)))
    for mo in motor_names:
        all_btns_manual.append(sg.Button("S", size=(4, 2)))


    all_txts_manual = []
    for mo in motor_names:
        all_txts_manual.append(sg.Input(default_text="255", size=4))
    for mo in motor_names:
        all_txts_manual.append(sg.Input(default_text="0", size=4))

    btn_init = sg.Button("INIT", size=7)


    # create layouts
    button_layout = []
    for i in range(len(motor_names)):
        button_layout.append(sg.Column([[sg.Text(motor_names[i])], 
                                        [all_btns_manual[i]], 
                                        [all_btns_manual[i+len(motor_names)]], 
                                        [all_btns_manual[i+2*len(motor_names)]], 
                                        [all_btns_manual[i+3*len(motor_names)]], 
                                        [all_txts_manual[i]],
                                        [all_btns_manual[i+4*len(motor_names)]],
                                        [all_txts_manual[i+len(motor_names)]]
                                        ], element_justification='center'))

    
    layout = [  [sg.Text("Initialize all motors:")], 
                [btn_init],
                [sg.Text("Checkbox to enable the manual drive mode:")], 
                [chkbx_manual],
                [button_layout]
                ]

    # create the window
    window = sg.Window("StepperMotorControl", layout, margins=(50, 50), finalize=True)

    for i in range(len(all_btns_manual)):
        all_btns_manual[i].bind("<ButtonPress-1>", motor_names[i%len(motor_names)] + "_press")
    
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
            ser.write(("INIT\n").encode())
        
        elif event.endswith("press"):
            i = 0
            for mo in motor_names:
                if mo in event:
                    if event[0] == "I":
                        send_init_cmd(mo)
                    elif event[0] == "Z":
                        send_zero_cmd(mo)
                    elif event[0] == "S":
                        send_set_pos_cmd(mo, all_txts_manual[i+len(motor_names)].get())
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
