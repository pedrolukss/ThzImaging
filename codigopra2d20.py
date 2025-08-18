from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QSpinBox, QLabel, QDoubleSpinBox, QComboBox, QGroupBox, QFormLayout
from pyvisa import ResourceManager
import pyqtgraph as pg
import sys
from PyQt5.QtCore import QTimer
import serial
import time 
import numpy as np
from scipy.interpolate import griddata
import pandas as pd
teste = pd.read_csv("ferror.csv")
 #Classe MockArduino (mantida como no seu exemplo anterior)
class MockArduino:
    def __init__(self, port, baudrate, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._is_open = True
        print(f"MockArduino: Conectado a {self.port} com baudrate {self.baudrate}")

    def write(self, data):
        if not self._is_open:
            print("MockArduino: Erro - Tentativa de escrita em porta fechada.")
        command = data.decode('utf-8').strip()
        print(f"MockArduino: Comando recebido: {command}")

    def close(self):
        self._is_open = False
        print(f"MockArduino: Conexão com {self.port} fechada.")

    @property
    def is_open(self):
        return self._is_open

# Classe LockIn (mantida como no seu exemplo anterior)
class LockIn:
    def __init__(self, address="GPIB0::10::INSTR"):
        #self.rm = ResourceManager()
        #self.device = self.rm.open_resource(address)
        self.sintdados = teste['y0000']
        self.indexagr = -1
    def X(self):
        self.indexagr += 1
        if self.indexagr >= len(self.sintdados):
            self.indexagr = 0
        return self.sintdados[self.indexagr] #float(self.device.query('OUTP?1'))

class RealTimePlot(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time PyQtGraph Plot - Varredura Linear e Raster")
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # --- Área do Gráfico ---
        self.imv = pg.ImageView(view=pg.PlotItem())
        main_layout.addWidget(self.imv)
        cm = pg.colormap.get('CET-L9') 
        self.imv.setColorMap(cm)
        
        self.plot_item = self.imv.getView()
        self.plot_item.setLabels(bottom='Posição X (unidades)', left='Posição Y (unidades)')
        self.min_x_plot = -10 
        self.max_x_plot = 10
        self.min_y_plot = -10
        self.max_y_plot = 10
        self.plot_item.setXRange(self.min_x_plot, self.max_x_plot, padding=0)
        self.plot_item.setYRange(self.min_y_plot, self.max_y_plot, padding=0)
        self.counter_text = pg.TextItem(text = "Step ?/?", color=(200, 200, 200),anchor=(0, 0))
        
        
        
        # --- Controles em Abas ou GroupBoxes para organização ---
        controls_layout = QHBoxLayout()

        # --- Controles para Varredura Linear ---
        linear_scan_group = QGroupBox("Varredura Linear Simples")
        linear_layout = QFormLayout()

        self.iter_label = QLabel("Número de Movimentos:")
        self.iter_input = QSpinBox()
        self.iter_input.setRange(1, 1000)
        self.iter_input.setValue(10)
        linear_layout.addRow(self.iter_label, self.iter_input)
        
        self.mov_label = QLabel("Passos por Movimento X/Ponto X:") 
        self.mov_input = QSpinBox()
        self.mov_input.setRange(1, 1000000)
        self.mov_input.setValue(1)
        linear_layout.addRow(self.mov_label, self.mov_input)
        
        #self.mov_labelv = QLabel("Velocidade RPM:") 
        #self.mov_inputv = QSpinBox()
        #self.mov_inputv.setRange(1, 1000000)
        #self.mov_inputv.setValue(15)
        #linear_layout.addRow(self.mov_labelv, self.mov_inputv)
        
        self.dir_label = QLabel("Direção (Linear):")
        self.dir_box = QComboBox()
        self.dir_box.addItems(["F", "B","L","R"]) 
        linear_layout.addRow(self.dir_label, self.dir_box)
        
        self.wait_label = QLabel("Tempo de Espera (s):")
        self.wait_input = QDoubleSpinBox()
        self.wait_input.setRange(0.01, 60.0)
        self.wait_input.setSingleStep(0.1)
        self.wait_input.setValue(0.01) 
        linear_layout.addRow(self.wait_label, self.wait_input)

        self.start_button = QPushButton("Iniciar Varredura Linear")
        linear_layout.addRow(self.start_button)
        linear_scan_group.setLayout(linear_layout)
        controls_layout.addWidget(linear_scan_group)
        
        #self.vel_button = QPushButton("Trocar velocidade")
       # linear_scan_group.setLayout(linear_layout)
       # controls_layout.addWidget(linear_scan_group)
        

        # --- Controles para Varredura Raster ---
        raster_scan_group = QGroupBox("Varredura Raster (Serpentina)")
        raster_layout = QFormLayout()

        self.raster_x_points_input = QSpinBox()
        self.raster_x_points_input.setRange(1, 1000)
        self.raster_x_points_input.setValue(10) 
        raster_layout.addRow("Pontos na Linha X:", self.raster_x_points_input)

        self.raster_y_lines_input = QSpinBox()
        self.raster_y_lines_input.setRange(1, 1000)
        self.raster_y_lines_input.setValue(10) 
        raster_layout.addRow("Número de Linhas Y:", self.raster_y_lines_input)
        
        self.raster_mov_y_input = QSpinBox() 
        self.raster_mov_y_input.setRange(1, 1000000)
        self.raster_mov_y_input.setValue(1)
        raster_layout.addRow("Passos por Ponto Y:", self.raster_mov_y_input)

        self.start_raster_button = QPushButton("Iniciar Varredura Raster")
        raster_layout.addRow(self.start_raster_button)
        raster_scan_group.setLayout(raster_layout)
        controls_layout.addWidget(raster_scan_group)
        
        main_layout.addLayout(controls_layout)

        # --- Botões Globais de Controle ---
        global_buttons_layout = QHBoxLayout()
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.reset_plot_button = QPushButton("Resetar Plot e Posição")
        self.close_button = QPushButton("Close")
        global_buttons_layout.addWidget(self.stop_button)
        global_buttons_layout.addWidget(self.reset_plot_button)
        global_buttons_layout.addWidget(self.close_button)
        main_layout.addLayout(global_buttons_layout)
        
        # --- Inicialização de Dados e Estado ---
        # MOVE INITIALIZATION OF STATE VARIABLES EARLIER
        self.running = False
        self.is_raster_scanning = False 

        # Para varredura linear simples
        self.current_iteration = 0
        self.total_iterations = 0
        
        # Para varredura raster
        self.raster_target_x_points = 0
        self.raster_target_y_lines = 0
        self.raster_steps_per_x_point = 0
        self.raster_steps_per_y_point = 0
        self.raster_current_x_point_idx = 0 
        self.raster_current_y_line_idx = 0  
        self.raster_x_scan_direction_char = "F" 
        self.raster_y_scan_direction_char = "L" 
        self.current_raster_command_dir = "" 
        
        self.x, self.y = 0, 0 
        self.x_data, self.y_data, self.z_data = [], [], []
        
        self.intensity_grid = None
        self.grid_resolution_x = 100 
        self.grid_resolution_y = 100
        
        # Now call reset_plot_and_position as self.running is defined
        self.reset_plot_and_position(initial_setup=True) 

        self.lockin = LockIn()
       # self.arduino = serial.Serial('COM6', 9600, timeout=1)
       # time.sleep(2)  # Aguarda a estabilização da conexão
        self.arduino = MockArduino('COM_SIMULADA', 9600, timeout=1)
        self.start_button.clicked.connect(self.start_motor_routine)
        self.start_raster_button.clicked.connect(self.start_raster_scan)
        self.stop_button.clicked.connect(self.stop_motor_routine)
        self.reset_plot_button.clicked.connect(self.reset_plot_and_position)
        self.close_button.clicked.connect(self.close_application)
        #self.vel_button.clicked.connect(self.velchange)
        
    def reset_plot_and_position(self, initial_setup=False):
        if self.running: # Now self.running exists
            self.stop_motor_routine()
        
        self.x = 0 
        self.y = 0 
        self.x_data = [self.x] 
        self.y_data = [self.y]
        self.z_data = [0] 

        if not initial_setup:
            self.record_measurement_and_update_coords(initial_point=True) 
            self.update_image_display() 
        else: 
            self.imv.clear()
            self.grid_x_1d = np.linspace(self.min_x_plot, self.max_x_plot, self.grid_resolution_x)
            self.grid_y_1d = np.linspace(self.min_y_plot, self.max_y_plot, self.grid_resolution_y)
            self.X_mesh, self.Y_mesh = np.meshgrid(self.grid_x_1d, self.grid_y_1d)
            
        print("Plot e posição lógica resetados para (0,0).")


    def record_measurement_and_update_coords(self, initial_point=False):
        effective_direction = ""
        if self.is_raster_scanning:
            effective_direction = self.current_raster_command_dir 
        elif self.running: 
            effective_direction = self.dir_box.currentText()

        if not initial_point: 
            if effective_direction == "F":
                self.x += 1
            elif effective_direction == "B":
                self.x -= 1
            
            if effective_direction == "L": 
                self.y += 1
            elif effective_direction == "R": 
                self.y -= 1
        
        current_z = self.lockin.X()
        
        if initial_point and len(self.x_data) > 0 and self.x_data[-1] == self.x and self.y_data[-1] == self.y:
            self.z_data[-1] = current_z 
        else:
            self.x_data.append(self.x)
            self.y_data.append(self.y)
            self.z_data.append(current_z)

        print(f"Gravado: ({self.x}, {self.y}), Z: {current_z}. Dir Efetiva: '{effective_direction}'")

    def update_image_display(self):
        if not self.x_data: 
            self.imv.clear()
            self.grid_x_1d = np.linspace(self.min_x_plot, self.max_x_plot, self.grid_resolution_x)
            self.grid_y_1d = np.linspace(self.min_y_plot, self.max_y_plot, self.grid_resolution_y)
            self.X_mesh, self.Y_mesh = np.meshgrid(self.grid_x_1d, self.grid_y_1d)
            return

        current_min_x_data, current_max_x_data = min(self.x_data), max(self.x_data)
        current_min_y_data, current_max_y_data = min(self.y_data), max(self.y_data)

        expanded_min_x = min(self.min_x_plot, current_min_x_data)
        expanded_max_x = max(self.max_x_plot, current_max_x_data)
        expanded_min_y = min(self.min_y_plot, current_min_y_data)
        expanded_max_y = max(self.max_y_plot, current_max_y_data)
        
        padding_x = (expanded_max_x - expanded_min_x) * 0.05 if expanded_max_x > expanded_min_x else 1
        padding_y = (expanded_max_y - expanded_min_y) * 0.05 if expanded_max_y > expanded_min_y else 1

        view_min_x = expanded_min_x - padding_x
        view_max_x = expanded_max_x + padding_x
        view_min_y = expanded_min_y - padding_y
        view_max_y = expanded_max_y + padding_y

        self.plot_item.setXRange(view_min_x, view_max_x, padding=0)
        self.plot_item.setYRange(view_min_y, view_max_y, padding=0)
        
        self.grid_x_1d = np.linspace(view_min_x, view_max_x, self.grid_resolution_x)
        self.grid_y_1d = np.linspace(view_min_y, view_max_y, self.grid_resolution_y)
        self.X_mesh, self.Y_mesh = np.meshgrid(self.grid_x_1d, self.grid_y_1d)

        if len(self.x_data) >= 3:
            current_points = np.column_stack((self.x_data, self.y_data))
            current_values = np.array(self.z_data)
            df = pd.DataFrame({ 'x':self.x_data, 'y':self.y_data, 'z':self.z_data})
            try:
                #fill_val = np.nanmin(current_values) if not np.all(np.isnan(current_values)) and len(current_values)>0 else 0
                self.intgr = df.pivot(index = 'x', columns = 'y', values = 'z')
                self.intensity_grid = self.intgr.values
#griddata(current_points, current_values,
                                       #        (self.X_mesh, self.Y_mesh),
                                        #       method='cubic', fill_value=fill_val)
                #self.intensity_grid[np.isnan(self.intensity_grid)] = fill_val 
            except Exception as e:
                print(f"Erro no griddata: {e}")
                self.imv.clear()
                return

            self.imv.setImage(self.intensity_grid, autoRange=False, autoLevels=True,
                                pos=[self.grid_x_1d[0], self.grid_y_1d[0]],
                                scale=[(self.grid_x_1d[-1]-self.grid_x_1d[0])/self.grid_resolution_x,
                                       (self.grid_y_1d[-1]-self.grid_y_1d[0])/self.grid_resolution_y])
        elif len(self.x_data) > 0: 
            base_val = np.nanmin(self.z_data) if not np.all(np.isnan(self.z_data)) and len(self.z_data) > 0 else 0
            temp_grid = np.full_like(self.X_mesh, base_val, dtype=float) 
            for i in range(len(self.x_data)):
                ix = np.argmin(np.abs(self.grid_x_1d - self.x_data[i]))
                iy = np.argmin(np.abs(self.grid_y_1d - self.y_data[i]))
                if 0 <= ix < self.grid_resolution_x and 0 <= iy < self.grid_resolution_y:
                        temp_grid[iy, ix] = self.z_data[i]
            self.imv.setImage(temp_grid.T, autoRange=False, autoLevels=True,
                                  pos=[self.grid_x_1d[0], self.grid_y_1d[0]],
                                  scale=[(self.grid_x_1d[-1]-self.grid_x_1d[0])/self.grid_resolution_x,
                                         (self.grid_y_1d[-1]-self.grid_y_1d[0])/self.grid_resolution_y])
        else:
            self.imv.clear()
    #def velchange(self):
      #  speed = self.mov_inputv.value()
       # self.arduino.write(f"V{speed}\n".encode('utf-8'))
        
    def start_motor_routine(self):
        if self.running:
            print("Uma rotina já está em execução.")
            return
        
        self.reset_plot_and_position() 

        self.current_iteration = 0
        self.total_iterations = self.iter_input.value()
        self.is_raster_scanning = False
        self.running = True
        
        self.start_button.setEnabled(False)
        self.start_raster_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        
        if self.total_iterations > 0:
            QTimer.singleShot(10, self.motor_sequence_step) 
        else:
            print("Nenhuma iteração para varredura linear.")
            self.stop_motor_routine()
            
    def motor_sequence_step(self): 
        if not self.running or self.is_raster_scanning:
            return
        
        if self.current_iteration < self.total_iterations:
            direction = self.dir_box.currentText()
            steps = self.mov_input.value()
            self.arduino.write(f"{direction}{steps}\n".encode('utf-8'))
            print(f"Linear: Iteração {self.current_iteration+1}/{self.total_iterations}. Movendo {direction} por {steps} passos.")
            movement_time = self.mov_input.value() / 100
            delay_time_ms = self.wait_input.value() + movement_time
            QTimer.singleShot(int(delay_time_ms*1000), self.measure_and_continue_linear)
        else:
            print("Rotina linear completa.")
            self.stop_motor_routine()
            
    def measure_and_continue_linear(self): 
        if not self.running or self.is_raster_scanning:
            return
        
        self.record_measurement_and_update_coords() 
        self.update_image_display()
        
        self.current_iteration += 1
        self.motor_sequence_step()

    def start_raster_scan(self):
        if self.running:
            print("Uma rotina já está em execução.")
            return

        self.raster_target_x_points = self.raster_x_points_input.value()
        self.raster_target_y_lines = self.raster_y_lines_input.value()
        self.raster_steps_per_x_point = self.mov_input.value() 
        self.raster_steps_per_y_point = self.raster_mov_y_input.value()
        if self.raster_target_x_points <= 0 or self.raster_target_y_lines <= 0:
            print("Pontos X e Linhas Y devem ser maiores que 0 para varredura raster.")
            return

        self.reset_plot_and_position()
        
        self.raster_current_x_point_idx = 1 
        self.raster_current_y_line_idx = 0  
        self.raster_x_scan_direction_char = "F" 
        self.raster_y_scan_direction_char = "R" 

        self.is_raster_scanning = True
        self.running = True
        self.start_button.setEnabled(False)
        self.start_raster_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        
        print(f"Iniciando varredura raster: {self.raster_target_x_points}x{self.raster_target_y_lines} pontos.")
        QTimer.singleShot(10, self.raster_control_loop) 

    def raster_control_loop(self): 
        if not self.running or not self.is_raster_scanning:
            return

        if self.raster_current_y_line_idx >= self.raster_target_y_lines:
            print("Varredura Raster completa.")
            self.stop_motor_routine()
            return

        if self.raster_current_x_point_idx >= self.raster_target_x_points:
            self.raster_current_y_line_idx += 1
            if self.raster_current_y_line_idx >= self.raster_target_y_lines: 
                print("Varredura Raster completa (fim da última linha Y).")
                self.stop_motor_routine()
                return
            
            self.current_raster_command_dir = self.raster_y_scan_direction_char
            motor_steps_y = self.raster_steps_per_y_point
            self.arduino.write(f"{self.current_raster_command_dir}{motor_steps_y}\n".encode('utf-8'))
            print(f"Raster: Movendo para próxima linha Y ({self.current_raster_command_dir}), {self.raster_current_y_line_idx+1}/{self.raster_target_y_lines}.")
            
            self.raster_current_x_point_idx = 0 
            self.raster_x_scan_direction_char = "B" if self.raster_x_scan_direction_char == "F" else "F" 
        else:
            self.current_raster_command_dir = self.raster_x_scan_direction_char
            motor_steps_x = self.raster_steps_per_x_point
            self.arduino.write(f"{self.current_raster_command_dir}{motor_steps_x}\n".encode('utf-8'))
            print(f"Raster: Linha Y {self.raster_current_y_line_idx+1}, Ponto X {self.raster_current_x_point_idx+1}/{self.raster_target_x_points} ({self.current_raster_command_dir}).")

        movement_time = self.mov_input.value() / 100
        delay_time_ms = self.wait_input.value() + movement_time
        QTimer.singleShot(int(delay_time_ms*1000), self.measure_and_continue_raster_scan)

    def measure_and_continue_raster_scan(self):
        if not self.running or not self.is_raster_scanning:
            return
        
        self.record_measurement_and_update_coords() 
        self.update_image_display()
        
        self.raster_current_x_point_idx += 1  
        
        QTimer.singleShot(10, self.raster_control_loop) 

    def stop_motor_routine(self):
        self.running = False
        self.is_raster_scanning = False 
        self.start_button.setEnabled(True)
        self.start_raster_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        newdf = {'X':self.x_data,'Y':self.y_data,'Z(intensidade)':self.z_data}
        dadosf = pd.DataFrame(data = newdf)
        dadosf.to_csv('dadosim.csv')
        gridex = pd.DataFrame(data = self.intensity_grid)
        gridex.to_csv('gridex.csv')
        print("Rotina interrompida/finalizada.")
        
    def close_application(self):
        self.stop_motor_routine()
        if hasattr(self, 'arduino') and self.arduino.is_open:
            self.arduino.close()
        self.close()

if __name__ == "__main__":
    app = QApplication.instance()
    if not app:
        app = QApplication(sys.argv)
    main_win = RealTimePlot()
    main_win.showMaximized() 
    sys.exit(app.exec_())

