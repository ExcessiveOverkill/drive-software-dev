import serial, time
import numpy as np

import tkinter as tk
from tkinter import messagebox

GAIN_LIMITS = {
    "p" : [0, 100],
    "i" : [0, 10],
    "d" : [0, 5]
}

def scaleToBinary(min, max, value, bits) -> int:
    """converts an input range and value to an output integer such that the output will be zero when the value is at the minimum and will be 2^bit when the value is at the maximum"""
    assert value >= min and value <= max

    output = (value-min)/(max-min) * (2**(bits)-1)
    return int(output)

class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Position Control")
        
        # Get screen width and height
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        
        # Set window size to fill the screen
        #self.root.geometry(f"{screen_width}x{screen_height}")
        self.root.geometry(f"{1920}x{1080}")
        
        self.is_running = False
        self.angle_var = tk.DoubleVar()
        self.p_gain_var = tk.DoubleVar()
        self.i_gain_var = tk.DoubleVar()
        self.d_gain_var = tk.DoubleVar()
        
        self.p_gain_enabled = False
        self.i_gain_enabled = False
        self.d_gain_enabled = False

        self.gainEnabledColor = "gray95"
        self.gainDisabledColor = "gray70"

        self.motorStateFonts = {
            "on" : ("Arial", 55, "bold"),
            "off" : ("Arial", 55, "bold")
        }
        
        self.create_widgets()
        
    def create_widgets(self):

        onOffFont = ("Arial", 25, "bold")
        defaultFont = ("Arial", 20)
        infoFont = ("Arial", 20)

        # info side panel
        infoPanelFrame = tk.Frame(self.root)
        infoPanelFrame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=10, pady=10)

        # info text

        infoText = "This is a demonstration to show the effects of different gain values in a PID control loop. Use the sliders and buttons to adjust gains and the desired position and see how the motor reacts."
        infoLabel = tk.Label(infoPanelFrame, text=infoText, font=infoFont, wraplength=400, justify="left")
        infoLabel.pack(side=tk.TOP, fill=tk.X, expand=False, padx=10, pady=10)

        # On and Off buttons
        onOffButtonFrame = tk.Frame(infoPanelFrame)
        onOffButtonFrame.pack(side=tk.TOP, fill=tk.X, expand=False, padx=10, pady=10)
        self.on_button = tk.Button(onOffButtonFrame, text="Start", font=onOffFont, width=10, command=self.start_motor)
        self.on_button.pack(side=tk.LEFT, padx=10, pady=10)
        self.off_button = tk.Button(onOffButtonFrame, text="Stop", font=onOffFont, width=10, command=self.stop_motor, state=tk.DISABLED)
        self.off_button.pack(side=tk.LEFT, padx=10, pady=10)

        # Motor on/off label
        self.motor_label = tk.Label(infoPanelFrame, text="MOTOR OFF", font=self.motorStateFonts["off"])
        self.motor_label.pack(side=tk.TOP, padx=10, pady=10)
        
        # Desired motor angle slider
        angle_frame = tk.Frame(self.root, borderwidth=5, relief=tk.GROOVE)
        angle_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.angle_label = tk.Label(angle_frame, text="Desired Angle:", font=defaultFont)
        self.angle_label.pack(side=tk.TOP, padx=10, pady=10)
        self.angle_slider = tk.Scale(angle_frame, from_=0, to=360, orient=tk.HORIZONTAL, variable=self.angle_var, command=self.update_desired_angle, showvalue=False, resolution=.1, width=30)
        self.angle_slider.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.angle_readout = tk.Label(angle_frame, textvariable=self.angle_var, font=defaultFont)
        self.angle_readout.pack(side=tk.TOP, padx=10, pady=10)
        
        # P Gain control box
        self.p_gain_frame = tk.Frame(self.root, borderwidth=5, relief=tk.GROOVE, bg=self.gainDisabledColor)
        self.p_gain_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.p_gain_label = tk.Label(self.p_gain_frame, text="P Gain: DISABLED", font=defaultFont, bg=self.gainDisabledColor)
        self.p_gain_label.pack(side=tk.TOP, padx=10, pady=10)
        self.p_gain_slider = tk.Scale(self.p_gain_frame, from_=GAIN_LIMITS["p"][0], to=GAIN_LIMITS["p"][1], orient=tk.HORIZONTAL, variable=self.p_gain_var, command=self.update_p_gain, showvalue=False, resolution=(GAIN_LIMITS["p"][1]-GAIN_LIMITS["p"][0])/1000, width=30, highlightthickness=0, bg=self.gainDisabledColor)
        self.p_gain_slider.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.p_gain_readout = tk.Label(self.p_gain_frame, textvariable=self.p_gain_var, font=defaultFont, bg=self.gainDisabledColor)
        self.p_gain_readout.pack(side=tk.TOP, padx=10, pady=10)
        self.p_gain_enable_button = tk.Button(self.p_gain_frame, text="Enable P Gain", bg=self.gainDisabledColor, font=defaultFont, width=15, command=self.toggle_p_gain)
        self.p_gain_enable_button.pack(side=tk.TOP, padx=10, pady=10)
        
        # I Gain control box
        self.i_gain_frame = tk.Frame(self.root, borderwidth=5, relief=tk.GROOVE, bg=self.gainDisabledColor)
        self.i_gain_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.i_gain_label = tk.Label(self.i_gain_frame, text="I Gain: DISABLED", font=defaultFont, bg=self.gainDisabledColor)
        self.i_gain_label.pack(side=tk.TOP, padx=10, pady=10)
        self.i_gain_slider = tk.Scale(self.i_gain_frame, from_=GAIN_LIMITS["i"][0], to=GAIN_LIMITS["i"][1], orient=tk.HORIZONTAL, variable=self.i_gain_var, command=self.update_i_gain, showvalue=False, resolution=(GAIN_LIMITS["i"][1]-GAIN_LIMITS["i"][0])/1000, width=30, highlightthickness=0, bg=self.gainDisabledColor)
        self.i_gain_slider.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.i_gain_readout = tk.Label(self.i_gain_frame, textvariable=self.i_gain_var, font=defaultFont, bg=self.gainDisabledColor)
        self.i_gain_readout.pack(side=tk.TOP, padx=10, pady=10)
        self.i_gain_enable_button = tk.Button(self.i_gain_frame, text="Enable I Gain", bg=self.gainDisabledColor, font=defaultFont, width=15, command=self.toggle_i_gain)
        self.i_gain_enable_button.pack(side=tk.TOP, padx=10, pady=10)
        
        # D Gain control box
        self.d_gain_frame = tk.Frame(self.root, borderwidth=5, relief=tk.GROOVE, bg=self.gainDisabledColor)
        self.d_gain_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.d_gain_label = tk.Label(self.d_gain_frame, text="D Gain: DISABLED", font=defaultFont, bg=self.gainDisabledColor)
        self.d_gain_label.pack(side=tk.TOP, padx=10, pady=10)
        self.d_gain_slider = tk.Scale(self.d_gain_frame, from_=GAIN_LIMITS["d"][0], to=GAIN_LIMITS["d"][1], orient=tk.HORIZONTAL, variable=self.d_gain_var, command=self.update_d_gain, showvalue=False, resolution=(GAIN_LIMITS["d"][1]-GAIN_LIMITS["d"][0])/1000, width=30, highlightthickness=0, bg=self.gainDisabledColor)
        self.d_gain_slider.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.d_gain_readout = tk.Label(self.d_gain_frame, textvariable=self.d_gain_var, font=defaultFont, bg=self.gainDisabledColor)
        self.d_gain_readout.pack(side=tk.TOP, padx=10, pady=10)
        self.d_gain_enable_button = tk.Button(self.d_gain_frame, text="Enable D Gain", bg=self.gainDisabledColor, font=defaultFont, width=15, command=self.toggle_d_gain)
        self.d_gain_enable_button.pack(side=tk.TOP, padx=10, pady=10)
        
    def start_motor(self):
        if not self.is_running:
            self.is_running = True
            self.on_button.config(state=tk.DISABLED)
            self.off_button.config(state=tk.NORMAL)
            self.motor_label.config(text="MOTOR ON", font=self.motorStateFonts["on"])
    
    def stop_motor(self):
        if self.is_running:
            self.is_running = False
            self.on_button.config(state=tk.NORMAL)
            self.off_button.config(state=tk.DISABLED)
            self.motor_label.config(text="MOTOR OFF", font=self.motorStateFonts["off"])
 
    def update_desired_angle(self, value):
        desired_angle = self.angle_var.get()
        self.angle_readout.config(text=f"{desired_angle:.2f}")
    
    def update_p_gain(self, value):
        p_gain = self.p_gain_var.get()
        if self.p_gain_enabled:
            self.p_gain_readout.config(text=f"{p_gain:.2f}")
    
    def update_i_gain(self, value):
        i_gain = self.i_gain_var.get()
        if self.i_gain_enabled:
            self.i_gain_readout.config(text=f"{i_gain:.2f}")
    
    def update_d_gain(self, value):
        d_gain = self.d_gain_var.get()
        if self.d_gain_enabled:
            self.d_gain_readout.config(text=f"{d_gain:.2f}")
    
    def toggle_p_gain(self):
        self.p_gain_enabled = not self.p_gain_enabled
        if self.p_gain_enabled:
            self.p_gain_enable_button.config(text="Disable P Gain", bg=self.gainEnabledColor)
            self.p_gain_frame.config(bg=self.gainEnabledColor)
            self.p_gain_label.config(bg=self.gainEnabledColor, text="P Gain: ENABLED")
            self.p_gain_readout.config(bg=self.gainEnabledColor)
            self.p_gain_slider.config(bg=self.gainEnabledColor)
        else:
            self.p_gain_enable_button.config(text="Enable P Gain", bg=self.gainDisabledColor)
            self.p_gain_frame.config(bg=self.gainDisabledColor)
            self.p_gain_label.config(bg=self.gainDisabledColor, text="P Gain: DISABLED")
            self.p_gain_readout.config(bg=self.gainDisabledColor)
            self.p_gain_slider.config(bg=self.gainDisabledColor)
    
    def toggle_i_gain(self):
        self.i_gain_enabled = not self.i_gain_enabled
        if self.i_gain_enabled:
            self.i_gain_enable_button.config(text="Disable I Gain", bg=self.gainEnabledColor)
            self.i_gain_frame.config(bg=self.gainEnabledColor)
            self.i_gain_label.config(bg=self.gainEnabledColor, text="I Gain: ENABLED")
            self.i_gain_readout.config(bg=self.gainEnabledColor)
            self.i_gain_slider.config(bg=self.gainEnabledColor)
        else:
            self.i_gain_enable_button.config(text="Enable I Gain", bg=self.gainDisabledColor)
            self.i_gain_frame.config(bg=self.gainDisabledColor)
            self.i_gain_label.config(bg=self.gainDisabledColor, text="D Gain: DISABLED")
            self.i_gain_readout.config(bg=self.gainDisabledColor)
            self.i_gain_slider.config(bg=self.gainDisabledColor)

    def toggle_d_gain(self):
        self.d_gain_enabled = not self.d_gain_enabled
        if self.d_gain_enabled:
            self.d_gain_enable_button.config(text="Disable D Gain", bg=self.gainEnabledColor)
            self.d_gain_frame.config(bg=self.gainEnabledColor)
            self.d_gain_label.config(bg=self.gainEnabledColor, text="D Gain: ENABLED")
            self.d_gain_readout.config(bg=self.gainEnabledColor)
            self.d_gain_slider.config(bg=self.gainEnabledColor)
        else:
            self.d_gain_enable_button.config(text="Enable D Gain", bg=self.gainDisabledColor)
            self.d_gain_frame.config(bg=self.gainDisabledColor)
            self.d_gain_label.config(bg=self.gainDisabledColor, text="D Gain: DISABLED")
            self.d_gain_readout.config(bg=self.gainDisabledColor)
            self.d_gain_slider.config(bg=self.gainDisabledColor)
       



if __name__ == "__main__":


    updateFrequency = 10    # Hz, rate at which to send new serial data

    try:
        nucleo = serial.Serial("COM4", 115200, timeout=.1)
    except Exception as e:
        print("cound not connect to serial port", e)
        nucleo = None

    
    # Create the main window
    root = tk.Tk()
    app = MotorControlApp(root)

    lastUpdateTime = 0

    packetLength = 0

    while True:
        root.update()

        if time.time() > lastUpdateTime + (1/updateFrequency):  # send data to nucelo
            lastUpdateTime = time.time()

            sendPacket = ""

            # 8 bits for enabling/disabling the motor (and other future control behavior)
            if app.is_running:  
                sendPacket += "00000001"    #0-7
            else:
                sendPacket += "00000000"    #0-7

            scaledAngle = scaleToBinary(0, 360, app.angle_var.get(), 16)
            sendPacket += f"{scaledAngle:>016b}" #8-23
            
            scaledPgain = scaleToBinary(GAIN_LIMITS["p"][0], GAIN_LIMITS["p"][1], app.p_gain_var.get() * int(app.p_gain_enabled), 16)
            sendPacket += f"{scaledPgain:>016b}" #24-39

            scaledIgain = scaleToBinary(GAIN_LIMITS["i"][0], GAIN_LIMITS["i"][1], app.i_gain_var.get() * int(app.i_gain_enabled), 16)
            sendPacket += f"{scaledIgain:>016b}" #40-55

            scaledDgain = scaleToBinary(GAIN_LIMITS["d"][0], GAIN_LIMITS["d"][1], app.d_gain_var.get() * int(app.d_gain_enabled), 16)
            sendPacket += f"{scaledDgain:>016b}" #56-71

            #sendPacket += (8 - (len(sendPacket) % 8)) * "0"   # add zeros to make sure packet ends with a number of bits divisible by 8
            assert len(sendPacket) % 8 == 0
        
            convertedSendPacket = int(sendPacket, 2).to_bytes(len(sendPacket) // 8, byteorder='big')

            packetLength = len(sendPacket)

            # print(f"send data is: {sendPacket} {convertedSendPacket.hex(':')} ({len(sendPacket)//8} bytes)")

            if nucleo is not None:
                nucleo.write(convertedSendPacket)

        try:
            root.winfo_exists()
        except tk.TclError:
            print("GUI window closed")

            # send all zeros before exiting

            zeros = bytes([0] * (packetLength//8))

            print(f"send data is: {zeros.hex(':')} ({len(zeros)} bytes)")
            
            if nucleo is not None:
                nucleo.write(zeros)

            quit()


            