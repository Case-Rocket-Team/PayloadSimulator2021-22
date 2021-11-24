#Import our many, many libraries!
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter import filedialog
import ctypes
import os
from tkinter.constants import Y


appID = 'CRT.PayloadSimulator.alpha1'

#Freaky Windows Shit
ctypes.windll.shcore.SetProcessDpiAwareness(1) #Enables support for Hi-DPI displays.
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(appID) #Wizardry to make the taskbar icon to the main icon

#GUI Specific declarations
root = tk.Tk()
widget_var = tk.StringVar()

big_frame = ttk.Frame(root)
big_frame.pack(fill="both", expand=True)

#photo = tk.PhotoImage(file='C:\\Users\\catsl\\Documents\\Telemtry Dash\\refresh.png')


#def setIcon(): #Sets icon
#    root.iconbitmap('C:\\Users\\catsl\\Documents\\Telemtry Dash\\icon.ico')


def get_filepaths(directory):
    file_paths = []  # List which will store all of the full filepaths.

    # Walk the tree.
    for root, directories, files in os.walk(directory):
        for filename in files:
            # Join the two strings in order to form the full filepath.
            filepath = os.path.join(root, filename)
            file_paths.append(filepath)  # Add it to the list.

    return file_paths  # Self-explanatory.

def openFolder():
    try:
        folder_selected = filedialog.askdirectory()
        paths = get_filepaths(folder_selected)
        SimulationPicker['values'] = paths
        SimulationPicker.current(0)
    except:
        messagebox.showerror('Error','Unable to open folder')

def change_theme():
    # NOTE: The theme's real name is sun-valley-<mode>
    if root.tk.call("ttk::style", "theme", "use") == "sun-valley-dark":
        # Set light theme
        root.tk.call("set_theme", "light")
    else:
        # Set dark theme
        root.tk.call("set_theme", "dark")
    

#Create UI
root.tk.call("source", "C:\\Users\\catsl\\Documents\\Telemtry Dash\\Sun-Valley-ttk-theme-master\sun-valley.tcl")
root.tk.call("set_theme", "dark")
root.resizable(False, False)
root.title("Payload Simulator")
root.geometry("800x500")
#root.after(0,setIcon)

button = ttk.Button(big_frame, text="Start Simulation",style="Accent.TButton")
button.place(x=30,y=425)
button.config(padding=10)
button.config(width=65)

filepicker = ttk.Button(big_frame, text="Open Folder",command=openFolder,style="Accent.TButton")
filepicker.place(x=625,y=25)
filepicker.config(width=12)

SimulationPicker = ttk.Combobox(big_frame,textvariable=widget_var)
SimulationPicker.config(width=50)
SimulationPicker.place(x=25,y=25)
SimulationPicker['state'] = 'readonly'
SimulationPicker.set("Open folder with simulation data")

lightdark = ttk.Button(big_frame, text="Switch Theme",command=change_theme)
lightdark.place(x=25,y=375)
param1 = ttk.Spinbox(big_frame,from_=0, to=100, increment=0.5)
param1.config(width=48)
param1.place(x=25,y=100)
param1.set("Paramater 1")

param2 = ttk.Spinbox(big_frame,from_=0, to=100, increment=0.5)
param2.config(width=48)
param2.place(x=25,y=175)
param2.set("Paramater 2")

param3 = ttk.Scale(big_frame,from_=100,to=0,length=500)
param3.config()
param3.place(x=25,y=250)
root.mainloop()