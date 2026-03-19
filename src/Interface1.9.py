# Welding Machine Control Interface
# Copyright (C) 2025  Jonas Frank Reis
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License, version 3,
# as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox, QFileDialog
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import sys
import serial
import time
import threading
from serial.tools import list_ports
from uiInterfaceFinal import Ui_Form
from PyQt5.QtGui import QDoubleValidator

import matplotlib.pyplot as plt
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader
import tempfile

def list_ports_func():
    available_ports = list_ports.comports()
    port_list = []
    for port in available_ports:
        description = f"{port.device} - {port.description}"
        port_list.append(description)
    return port_list

# Function to detect if an Arduino is connected to a port
def detect_arduino():
    for port in list_ports.comports():
        if "Arduino" in port.description or "CH340" in port.description:
            return port.device
    return None

# Arduino control class
# Acts as a thread, allowing simultaneous execution with the main code
class ArduinoSerialReader(threading.Thread):
    def __init__(self, port_name, baudrate=9600): # Class initialization
        super().__init__()
        self.arduino = None
        self.port = port_name
        self.baudrate = baudrate
        self.running = True
        self.data = {
            'settemp': 0,
            'temp': 0.0,
            'load': 0.0,
            'ok': 0
        }
        self.lock = threading.Lock()

    # Connection and data reading function
    def run(self):
        try:
            self.arduino = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            print("Connected to Arduino.")
        except Exception as e:
            print("Error connecting to Arduino:", e)
            return

        while self.running:
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode(errors='ignore').strip()
                try:
                    ok_str, load_str = line.split(",")
                    with self.lock:
                        self.data['ok'] = float(ok_str)
                        self.data['load'] = float(load_str)
                except Exception:
                    print("Invalid data from Arduino:", line)

    # Function to send commands
    def send_command(self, command):
        try:
            if self.arduino and self.arduino.is_open:
                self.arduino.write((command + "\n").encode())
                print(f"🔁 Sent to Arduino: {command}")
        except Exception as e:
            print(f"Error sending command to Arduino: {e}")

    # Function to stop the thread
    def stop(self):
        self.running = False
        if self.arduino:
            self.arduino.close()

# Power Supply control class (DPM)
class DPMPowerSupplyReader(threading.Thread):
    def __init__(self, port_name, baudrate=115200): # Class initialization
        super().__init__()
        self.ser = None
        self.port = port_name
        print({self.port})
        self.baudrate = baudrate
        self.running = True
        self.voltage_read = None
        self.lock = threading.Lock()

    def run(self): # Connection and data polling function
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(0.5)
            print("Connected to DPM power supply.")
        except Exception as e:
            print("Error connecting to DPM power supply:", e)
            return

        while self.running:
            response = self.send_ascii("01r30=0")
            if response and response.startswith(":01r30="):
                try:
                    value = float(response.split("=")[1])
                    with self.lock:
                        self.voltage_read = value / 100
                except:
                    pass
            time.sleep(1)

    # Function to send values to power supply
    def send_ascii(self, command):
        try:
            formatted_command = f":{command},\r\n"
            self.ser.write(formatted_command.encode())
            time.sleep(0.2)
            return self.ser.readline().decode().strip()
        except:
            return ""

    # Set voltage based on interface input
    def set_voltage(self, value_v):
        int_value = float(value_v * 100)
        self.send_ascii(f"01w10={int_value}")

    # Set current based on interface input
    def set_current(self, value_a):
        int_value = float(value_a * 1000)
        self.send_ascii(f"01w11={int_value}")

    # Initialize output
    def turn_on_output(self):
        self.send_ascii("01w12=1")

    # Disable output
    def turn_off_output(self):
        self.send_ascii("01w12=0")

    # Stop the thread
    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

# Interface Control Class (Main logic)
class Interface(QtWidgets.QWidget):
    def __init__(self, serial_reader, dpm_supply): # Interface initialization
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        validator = QDoubleValidator(0.00, 999.99, 2)
        validator.setNotation(QDoubleValidator.StandardNotation)
        self.ui.lineEdit_pressao.setValidator(validator)
        
        self.reader = serial_reader
        self.supply = dpm_supply

        self.timestamps = []
        self.voltage_values = []
        self.power_values = []
        self.start_time = None
        self.limit_time_min = 0
        self.supply_already_off = False

        self.ui.widget.setBackground((0, 0, 0, 0))
        self.voltage_curve = self.ui.widget.plot(pen='b', name='Voltage')
        self.power_curve = self.ui.widget.plot(pen='r', name='Power')
        self.ui.widget.setLabel('left', 'Value')
        self.ui.widget.setLabel('bottom', 'Time (s)')
        self.ui.widget.showGrid(x=True, y=True)
        self.ui.widget.setYRange(0, 1000)
        self.ui.widget.setXRange(0, 120)

        self.ui.btn_iniciar.clicked.connect(self.start_process)
        self.ui.btn_voltar.clicked.connect(self.back_screen)
        self.ui.btn_salvar.clicked.connect(self.save_config)
        self.ui.btn_carregar.clicked.connect(self.load_config)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_interface)

        ports = list_ports_func()
        self.ui.comboBox.addItems(ports)
        self.ui.comboBox_2.addItems(ports)

        self.previous_ports = ports
        self.port_timer = QtCore.QTimer()
        self.port_timer.timeout.connect(self.check_port_changes)
        self.port_timer.start(1000)

    # Function to update combobox for Arduino port selection
    def update_arduino_combo(self):
        current_ports = list_ports_func()
        if current_ports != self.previous_ports:
            self.ui.comboBox_2.blockSignals(True)
            selected_port = self.ui.comboBox_2.currentText()
            self.ui.comboBox_2.clear()
            self.ui.comboBox_2.addItems(current_ports)
            if selected_port in current_ports:
                self.ui.comboBox_2.setCurrentText(selected_port)
            self.ui.comboBox_2.blockSignals(False)
            self.previous_ports = current_ports

    # Function to update combobox for Power Supply port selection
    def update_supply_combo(self):
        current_ports = list_ports_func()
        if current_ports != self.previous_ports:
            self.ui.comboBox.blockSignals(True)
            selected_port = self.ui.comboBox.currentText()
            self.ui.comboBox.clear()
            self.ui.comboBox.addItems(current_ports)
            if selected_port in current_ports:
                self.ui.comboBox.setCurrentText(selected_port)
            self.ui.comboBox.blockSignals(False)
            self.previous_ports = current_ports

    def check_port_changes(self): 
        self.update_arduino_combo()
        self.update_supply_combo()

    def save_config(self):
        file_path, _ = QFileDialog.getSaveFileName(self, "Save File", "", "Text Files (*.txt)")
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as file:
                    file.write(self.ui.lineEdit_tensao_4.text().strip() + "\n") # Length
                    file.write(self.ui.lineEdit_tensao_3.text().strip() + "\n") # Width
                    file.write(self.ui.lineEdit_tensao.text().strip() + "\n")
                    file.write(self.ui.lineEdit_corrente.text().strip() + "\n")
                    file.write(self.ui.lineEdit_pressao.text().strip() + "\n")
                    file.write(self.ui.lineEdit_tempo.text().strip() + "\n")
                QMessageBox.information(self, "Success", "File saved successfully!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error saving file:\n{e}")

    def load_config(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open File", "", "Text Files (*.txt)")
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as file:
                    lines = [line.strip() for line in file.readlines()]
                    if len(lines) >= 6:
                        self.ui.lineEdit_tensao_4.setText(lines[0]) # Length
                        self.ui.lineEdit_tensao_3.setText(lines[1]) # Width
                        self.ui.lineEdit_tensao.setText(lines[2])
                        self.ui.lineEdit_corrente.setText(lines[3])
                        self.ui.lineEdit_pressao.setText(lines[4])
                        self.ui.lineEdit_tempo.setText(lines[5])
                        QMessageBox.information(self, "Success", "Values loaded successfully!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error loading file:\n{e}")

    def start_process(self): 
        try:
            supply_port_text = self.ui.comboBox.currentText()
            if not supply_port_text:
                QtWidgets.QMessageBox.warning(self, "Port Not Selected", "Select the Power Supply port before starting.")
                return
            
            arduino_port_text = self.ui.comboBox_2.currentText()
            if not arduino_port_text:
                QtWidgets.QMessageBox.warning(self, "Port Not Selected", "Select the Arduino port before starting.")
                return

            if supply_port_text == arduino_port_text:
                QtWidgets.QMessageBox.warning(self, "Conflict", "The same port is selected for both devices.")
                return            

            supply_port = supply_port_text.split(" - ")[0]
            arduino_port = arduino_port_text.split(" - ")[0]

            if self.supply:
                self.supply.stop()
                self.supply.join()

            if self.reader:
                self.reader.stop()
                self.reader.join()

            self.supply = DPMPowerSupplyReader(port_name=supply_port)
            self.supply.start()
            self.reader = ArduinoSerialReader(port_name=arduino_port)
            self.reader.start()
            time.sleep(5)

            voltage = float(self.ui.lineEdit_tensao.text())
            current = float(self.ui.lineEdit_corrente.text())
            pressure = float(self.ui.lineEdit_pressao.text())
            length = float(self.ui.lineEdit_tensao_4.text())
            width = float(self.ui.lineEdit_tensao_3.text())
            self.area = length * width
            process_time = float(self.ui.lineEdit_tempo.text())
            
            self.voltage_def = voltage
            self.current_def = current
            self.pressure_def = pressure
            
            print(f"[Start] Voltage: {voltage} V | Current: {current} A | Pressure: {pressure} MPa | Time: {process_time} s")
            self.reader.send_command(f"{pressure:.2f},{int(process_time)}")
            time.sleep(0.5)
            
            self.ui.stackedWidget.setCurrentIndex(1)
            current_load = self.reader.data['load']
            self.status_ok = self.reader.data['ok']
            
            while self.status_ok == 0:
                self.ui.label_pressao.setText(f"Pressure: {current_load:.2f} MPa")
                current_load = self.reader.data['load']
                self.status_ok = self.reader.data['ok']
            
            self.supply.set_voltage(voltage)
            self.supply.set_current(current)
            self.supply.turn_on_output()

            self.start_time = time.time()
            self.limit_time_min = process_time
            self.supply_already_off = False

            self.timestamps = []
            self.voltage_values = []
            self.power_values = []

            self.timer.start(1000)
            from datetime import datetime
            timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.data_filename = f"process_data_{timestamp_str}.txt"
            self.data_file = open(self.data_filename, "w", encoding="utf-8")
            self.data_file.write("Time(s)\tVoltage(V)\tCurrent(A)\tPower(W)\tPressure(MPa)\tTemp(°C)\n")
            
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Error", "Please fill all fields correctly!")

    def update_interface(self): 
        elapsed_time = (time.time() - self.start_time)

        with self.reader.lock:
            pressure = self.reader.data['load']
            temperature = self.reader.data['temp']

        voltage = None
        if self.supply:
            with self.supply.lock:
                voltage = self.supply.voltage_read
        
        if voltage is not None:
            current = self.current_def
            power = voltage * current
            power_density = power / self.area
            resistance = voltage / current
            
            self.timestamps.append(elapsed_time)
            self.voltage_values.append(voltage)
            self.power_values.append(power)

            self.voltage_curve.setData(self.timestamps, self.voltage_values)
            self.power_curve.setData(self.timestamps, self.power_values)
            
            if self.data_file:
                try:
                    self.data_file.write(
                        f"{elapsed_time:.2f}\t\t{voltage:.2f}\t\t{current:.2f}\t\t{power:.2f}\t\t{pressure:.2f}\t\t{temperature:.2f}\n"
                    )
                except Exception as e:
                    print(f"Error writing to TXT: {e}")

            self.ui.label_temp.setText(f"Voltage: {voltage:.2f} V")
            self.ui.label_temp_2.setText(f"Temperature: {temperature:.2f} °C")
            self.ui.label_pressao.setText(f"Pressure: {pressure:.2f} MPa")
            self.ui.label_tempo.setText(f"Time: {elapsed_time:.2f} s")
            self.ui.label_tempo_2.setText(f"Power: {power:.2f} W")
            self.ui.label_densidadep.setText(f"Power Density: {power_density:.2f} W/cm²")
            self.ui.label_resistividade.setText(f"Resistance: {resistance:.2f} Ω")

        if elapsed_time >= self.limit_time_min and not self.supply_already_off:
            print("Target time reached. Shutting down power supply...")
            if self.supply:
                self.supply.turn_off_output()
            self.supply_already_off = True
            self.timer.stop()
            self.save_plot_and_pdf()
            QtWidgets.QMessageBox.information(self, "Process Finished", "Target time reached. Supply disabled.\nPDF report saved.")
            if self.data_file:
                self.data_file.close()
                self.data_file = None    

    def save_plot_and_pdf(self):
        plt.figure()
        plt.plot(self.timestamps, self.voltage_values, color='blue')
        plt.xlabel('Time (s)')
        plt.ylabel('Voltage (V)')
        plt.title('Voltage over Time')

        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f_img:
            img_path = f_img.name
            plt.savefig(img_path)
            plt.close()

        from datetime import datetime
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        pdf_name = f"process_report_{timestamp}.pdf"
        c = canvas.Canvas(pdf_name, pagesize=A4)

        c.setFont("Helvetica-Bold", 14)
        c.drawString(50, 800, "Process Report")

        c.setFont("Helvetica", 12)
        c.drawString(50, 770, f"Total Time: {self.limit_time_min:.2f} s")
        c.drawString(50, 755, f"Set Voltage: {self.voltage_def:.2f} V")
        c.drawString(50, 740, f"Set Current: {self.current_def:.2f} A")
        c.drawString(50, 725, f"Set Pressure: {self.pressure_def:.2f} MPa")

        if self.voltage_values:
            final_voltage = self.voltage_values[-1]
            c.drawString(200, 770, f"Final Voltage: {final_voltage:.2f} V")

        with self.reader.lock:
            final_pressure = self.reader.data['load']
        c.drawString(200, 755, f"Final Pressure: {final_pressure:.2f} MPa")

        c.drawImage(ImageReader(img_path), 50, 400, width=500, height=300)
        c.save()
        print(f"PDF saved as {pdf_name}")

    def back_screen(self):
        self.timer.stop()
        if self.reader:
            self.reader.send_command("STOP")
        if self.supply:
            self.supply.turn_off_output()
        self.ui.stackedWidget.setCurrentIndex(0)
        if self.data_file:
            self.data_file.close()
            self.data_file = None  

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Interface(None, None)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        if window.reader:
            window.reader.stop()
        if window.supply:
            window.supply.stop()