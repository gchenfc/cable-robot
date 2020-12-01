# hello_psg.py

import PySimpleGUI as sg
import time
import threading
from robot import Robot
from enums import AxisStateEnum
import traceback

class MyGUI:
    def __init__(self, robot: Robot, exit_cb, sendMsgRaw):
        self.robot = robot
        self.exit_cb = exit_cb
        self.sendMsgRaw = sendMsgRaw

    def start(self):
        input_thread = threading.Thread(target=self.run)
        input_thread.daemon = True
        input_thread.start()

    def run(self):
        sg.theme_background_color('black')
        sg.theme_element_background_color('black')
        sg.theme_text_element_background_color('black')
        sg.theme_element_text_color('white')
        sg.theme_text_color('white')
        textkwargs = {'size': (20, 1),
                      'justification': 'center'}
        buttonkwargs = {'pad':(0,0), 
                        'auto_size_button':False, 
                        'size':(3,1)}
        # axis interface
        self.label_lasthb = [sg.Text("-"*5, **textkwargs) for i in range(4)]
        self.label_err = [sg.Text("-"*5, **textkwargs) for i in range(4)]
        self.label_state = [sg.Text("-"*5, **textkwargs) for i in range(4)]
        self.label_pos = [sg.Text("-"*5, **textkwargs) for i in range(4)]
        self.label_current = [sg.Text("-"*5, **textkwargs) for i in range(4)]
        def row(*args):
            return sg.Column([[*args]])
        def rowButtons(labels, keyBase, **kwargs):
            return row(*[sg.Button(label,
                                   key=(*keyBase, i),
                                   **{**buttonkwargs,
                                   **kwargs}) for i, label in enumerate(labels)])
        self.input_setpoint = [sg.InputText('0',
                                            key='setpoint{}'.format(axis),
                                            size=(7, 1)) for axis in range(4)]
        col_axes = [sg.Column([[sg.Text("Axis {}".format(axis), **textkwargs)],
                               [self.label_lasthb[axis]],
                               [self.label_err[axis]],
                               [self.label_state[axis]],
                               [self.label_pos[axis]],
                               [self.label_current[axis]],
                               [sg.Text('_'*20)],
                               [rowButtons(['IDLE', 'CALIB', 'CLOSED'], ('rs', axis), font='helvetica 8')],
                               [rowButtons(['Torque', 'Vel', 'Pos'], ('cm', axis))],
                               [row(self.input_setpoint[axis],
                                    sg.Button('Enter', key='set ax {}'.format(axis), size=(5, 1)))],
                               [sg.Text('_'*20)],
                               [rowButtons(['0', '1', '2', '3'], ('cal', axis), size=(1,1))]
                            ]) for axis in range(4)]
        col_labels = sg.Column([[sg.Text()],
                                [sg.Text("heartbeat")],
                                [sg.Text("error code")],
                                [sg.Text("state")],
                                [sg.Text("position")],
                                [sg.Text("current (meas)")],
                                [sg.Text('_'*15)],
                                [sg.Text("Requested State", size=(None, 2), pad=(5, 0))],
                                [sg.Text("Control Mode", size=(None, 2))],
                                [sg.Text("Setpoint:", pad=(5, 7))],
                                [sg.Text('_'*15)],
                                [sg.Text('Calibration')]
                            ], vertical_alignment='top')
        self.input_manual = sg.InputText(size=(15, 1), key='manualcommand') 
        
        def col_buttons(*args):
            return sg.Column([[sg.Button(label)] for label in args], vertical_alignment='top')
        controls = [col_buttons('Calculate Calibration', 'Export Calibration', 'Import Calibration'),
                    col_buttons('run_anticogging'),
                    col_buttons('ESTOP', 'Clear errors'),
                    col_buttons('reboot 0', 'reboot 1'),
                    col_buttons('QUIT')]
        layout = [[col_labels, *col_axes],
                  [sg.Text('_'*120)],
                  controls,
                  [sg.Text('_'*120)],
                  [sg.Text("Manual ASCII entry:"), self.input_manual, sg.Button("send", bind_return_key=True)],
                  [sg.Output(size=(120, 20), echo_stdout_stderr=True)]]

        # Create the window
        window = sg.Window("Demo", layout)

        # Create an event loop
        tstart = time.time()
        while True:
            event, values = window.read(timeout=20)
            # End program if user closes window or
            # presses the OK button
            if event == "QUIT" or event == sg.WIN_CLOSED:
                break
            else:
                try:
                    self.parseInput(event, values)
                except:
                    traceback.print_exc()
            tnow = time.time()
            tdur, tstart = tnow - tstart, tnow
            # print('dur: {}s'.format(tdur))
            try:
                self.updateStats()
            except:
                traceback.print_exc()

        window.close()

        self.exit_cb()
    
    def parseInput(self, event, values):
        if event == '__TIMEOUT__':
            return
        elif event == 'send':
            msg = values['manualcommand']
            print('sending {}'.format(msg))
            self.sendMsgRaw(msg)
            self.input_manual.Update('')
        elif event == 'ESTOP':
            self.robot.estop()
        elif event == 'Clear errors':
            self.robot.clear_errors()
        elif event == 'reboot 0':
            self.robot.reboot(0)
        elif event == 'reboot 1':
            self.robot.reboot(1)
        elif event == 'anticogging':
            print('NOT TESTED YET - DISABLING FOR SAFETY')
        elif isinstance(event, tuple):
            if event[0] == 'rs':
                STATE_OPTIONS = [AxisStateEnum.AXIS_STATE_IDLE,
                                    AxisStateEnum.AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
                                    AxisStateEnum.AXIS_STATE_CLOSED_LOOP_CONTROL]
                axis, state = event[1], STATE_OPTIONS[event[2]]
                print('request state of axis {} to {}: {}'.format(axis, state.value, state))
                self.robot.set_state(axis, state.value)
            elif event[0] == 'cm':
                axis, mode = event[1:]
                mode += 1
                print('request mode of axis {} to {}'.format(axis, mode))
                self.robot.set_ctrl_mode(axis, mode)
            elif event[0] == 'cal':
                axis, other_axis = event[1:]
                print('axis {} touched axis {}'.format(axis, other_axis))
                if self.robot.set_calib_length(axis, other_axis):
                    print('\tset succesful')
                else:
                    print("\tset unsuccessful")
            else:
                print(event, values)
        elif event[:-1] == 'set ax ':
            axis = int(event[-1])
            try:
                value = int(values['setpoint{}'.format(axis)])
            except:
                value = float(values['setpoint{}'.format(axis)])
            print('setpoint axis {} to {}'.format(axis, value))
            self.robot.set_setpoint(axis, value)
        elif event == 'Calculate Calibration':
            print('NOT IMPLEMENTED YET')
        elif event == 'Export Calibration':
            print(self.robot.export_calibration())
        else:
            print(event, values)

    def updateStats(self):
        tnow = time.time()
        for axis in range(4):
            ax = self.robot.axes[axis]
            hbdur = tnow - ax.last_hb_t
            self.label_lasthb[axis].Update(
                '{:.5f}'.format(hbdur),
                background_color='#{:02x}0000'.format(round(255*max(0, min(1, hbdur)))))
            self.label_err[axis].Update('{}'.format(ax.errorcode),
                background_color='black' if ax.errorcode == 0 else 'red')
            self.label_state[axis].Update('{}'.format(AxisStateEnum(ax.state).name[11:]))
            self.label_pos[axis].Update('{:.2f}'.format(ax.pos))
            self.label_current[axis].Update('{:.2f}'.format(ax.current_measured))

if __name__ == '__main__':
    def exit_cb():
        print('exiting!')
    gui = MyGUI(Robot(), exit_cb)
    gui.start()
    print('continuing...')
    time.sleep(1)
    print('main thread done')
