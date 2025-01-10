from src.config import config
import time
import cv2
import tkinter as tk
import serial
import numpy as np
import threading
import importlib
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class platformasterowania:
    def __init__(self, root, camera_frame, display_frame):
        # Inicjalizacja pozycji zerowych i parametrów ruchu
        self.root = root
        self.angles = config.zero_masz[:]  # Kopia pozycji zerowych
        self.speed = config.default_speed
        self.step_size = config.step_size
        self.przesuniecie_robocze = config.przesuniecie_robocze
        self.x = None
        self.y = None

        self.min_radius = 15  # Domyślny minimalny promień kulki
        self.max_radius = 50  # Domyślny maksymalny promień kulki

        self.missed_frames = 0  # Licznik klatek bez kulki
        self.max_missed_frames = 15  # Maksymalna liczba klatek bez kulki przed wyłączeniem auto
        self.auto_active = False


        self.kp_x = 0.05  # Wzmocnienie proporcjonalne dla osi X
        self.ki_x = 0.0  # Wzmocnienie całkujące dla osi X
        self.kd_x = 0.15  # Wzmocnienie różniczkujące dla osi X
        self.kp_y = 0.025  # Wzmocnienie proporcjonalne dla osi Y
        self.ki_y = 0.0  # Wzmocnienie całkujące dla osi Y
        self.kd_y = 0.08 # Wzmocnienie różniczkujące dla osi Y

        self.d_x = 0
        self.i_x = 0
        self.p_x = 0

        self.d_y = 0
        self.i_y = 0
        self.p_y = 0

        # Zmienne dla błędów PID
        self.previous_error_x = 0  # Poprzedni błąd dla osi X (D)
        self.previous_error_y = 0  # Poprzedni błąd dla osi Y (D)
        self.integral_x = 0  # Suma błędów dla osi X (I)
        self.integral_y = 0  # Suma błędów dla osi Y (I)

        self.Servo1_value = None
        self.Servo2_value = None
        self.Servo3_value = None
        self.Servo4_value = None

        # Stan aplikacji
        self.stol_w_poz_roboczej = False
        self.toggle_state_ruchyosiami = False
        self.toggle_state_ruchymanualne = False
        self.xz_value = 0
        self.yz_value = 0
        self.cap = cv2.VideoCapture(0)

        self.time_data = []  # Dane czasu
        self.x_position_data = []  # Pozycja X
        self.y_position_data = []  # Pozycja Y
        self.p_x_data = []  # Człon P dla X
        self.i_x_data = []  # Człon I dla X
        self.d_x_data = []  # Człon D dla X
        self.p_y_data = []  # Człon P dla Y
        self.i_y_data = []  # Człon I dla Y
        self.d_y_data = []  # Człon D dla Y
        self.start_time = time.time()  # Czas początkowy
        self.ani = None

        self.error_history_x = []
        self.error_history_y = []
        history_length = 5  # Liczba punktów do uwzględnienia

        if not self.cap.isOpened():
            print("Nie można otworzyć kamery")

        # Dodanie widżetu Tkinter do wyświetlania obrazu z kamery
        self.camera_label = tk.Label(camera_frame, bg="black")
        self.camera_label.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        # Dodanie widżetu tekstowego do wyświetlania pozycji kulki
        self.display_text = tk.Text(display_frame, wrap=tk.WORD, width=50, height=10)
        self.display_text.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        # Zainicjuj wywołanie aktualizacji kamery w GUI
        self.update_camera()        # Inicjalizacja połączenia z Arduino
        try:
            self.ser = serial.Serial(config.arduino_port, config.baud_rate, timeout=config.timeout)
            print(f"Połączono z Arduino na porcie: {config.arduino_port}")
        except Exception as e:
            print(f"Nie można połączyć się z Arduino na porcie: {config.arduino_port}")
            raise e

        # Oczekiwanie na gotowość Arduino
        self.wait_for_ready_message()
#region komunikacja
    def wait_for_ready_message(self, expected_message="Arduino gotowe do komunikacji"):
        print("Oczekiwanie na gotowość Arduino...")
        while True:
            if self.ser.in_waiting:
                response = self.ser.readline().decode().strip()
                print(f"Arduino: {response}")
                if expected_message in response:
                    print("Arduino zgłosiło gotowość!")
                    break

    def send_command(self, servo_id, angle):
        command = f"{servo_id} {angle}\n"
        self.ser.write(command.encode())
        self.display_text.insert(tk.END, f"Python: {command.strip()}\n")  # Wyświetlanie komendy
        self.display_text.see(tk.END)

    def read_response(self):
        time.sleep(0.1)
        while self.ser.in_waiting:
            response = self.ser.readline().decode().strip()
            self.display_text.insert(tk.END, f"Arduino: {response}\n")  # Wyświetlanie odpowiedzi
            self.display_text.see(tk.END)
#endregion

    def update_pid(self, parameter, value):
        if parameter == "kp_x":
            self.kp_x = value
        elif parameter == "ki_x":
            self.ki_x = value
        elif parameter == "kd_x":
            self.kd_x = value
        elif parameter == "kp_y":
            self.kp_y = value
        elif parameter == "ki_y":
            self.ki_y = value
        elif parameter == "kd_y":
            self.kd_y = value

        # Wyświetlenie zaktualizowanych wartości w sekcji tekstowej
        self.display_text.insert(tk.END, f"Zaktualizowano {parameter}: {value}\n")
        self.display_text.see(tk.END)

    def auto_mode(self):

        if not self.auto_active:
            return  # Wyjście, jeśli tryb automatyczny jest wyłączony

        # Obliczenie błędu (różnicy pozycji kulki względem środka)
        error_x = self.delta_x
        error_y = self.delta_y

        # Człon proporcjonalny (P)
        self.p_x = self.kp_x * error_x
        self.p_y = self.kp_y * error_y

        # Człon całkujący (I): Dodawanie błędów do sumy
        self.integral_x += error_x
        self.integral_y += error_y
        self.i_x = self.ki_x * self.integral_x
        self.i_y = self.ki_y * self.integral_y

        forget_factor = 0.1  # Im mniejsze, tym szybciej człon I "zapomina"
        self.i_x *= forget_factor
        self.i_y *= forget_factor
        self.i_x = max(min(self.i_x, 15), -15)
        self.i_y = max(min(self.i_y, 15), -15)

        # Aktualizacja list danych
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.x_position_data.append(self.delta_x)
        self.y_position_data.append(self.delta_y)
        self.p_x_data.append(self.p_x)
        self.i_x_data.append(self.i_x)
        self.d_x_data.append(self.d_x)
        self.p_y_data.append(self.p_y)
        self.i_y_data.append(self.i_y)
        self.d_y_data.append(self.d_y)

        # Ogranicz liczbę przechowywanych danych
        max_points = 100
        if len(self.time_data) > max_points:
            self.time_data.pop(0)
            self.x_position_data.pop(0)
            self.y_position_data.pop(0)
            self.p_x_data.pop(0)
            self.i_x_data.pop(0)
            self.d_x_data.pop(0)
            self.p_y_data.pop(0)
            self.i_y_data.pop(0)
            self.d_y_data.pop(0)

        if abs(error_x) < 10:  # Zakładamy, że błąd mniejszy niż 5 pikseli to "osiągnięty cel"
            self.integral_x = 0

        if abs(error_y) < 10:
            self.integral_y = 0


        # Dodaj obecny błąd do historii
        self.error_history_x.append(error_x)
        self.error_history_y.append(error_y)
        history_length = 4
        # Zachowaj tylko ostatnie N punktów
        if len(self.error_history_x) > history_length:
            self.error_history_x.pop(0)
        if len(self.error_history_y) > history_length:
            self.error_history_y.pop(0)

        # Oblicz średnią różniczkę
        if len(self.error_history_x) > 1:
            derivative_x = (self.error_history_x[-1] - self.error_history_x[0]) / (len(self.error_history_x) - 1)
            derivative_y = (self.error_history_y[-1] - self.error_history_y[0]) / (len(self.error_history_y) - 1)
        else:
            derivative_x = 0
            derivative_y = 0

        alpha = 0.6  # Współczynnik tłumienia dla filtra dolnoprzepustowego
        self.d_x = alpha * self.d_x + (1 - alpha) * self.kd_x * derivative_x
        self.d_y = alpha * self.d_y + (1 - alpha) * self.kd_y * derivative_y


        # Zaktualizowanie poprzednich błędów na potrzeby kolejnych obliczeń różniczkowych
        self.previous_error_x = error_x
        self.previous_error_y = error_y
        if not self.auto_active:
            self.p_y = 0
            self.i_y = 0
            self.d_y = 0
            self.p_x = 0
            self.i_x = 0
            self.d_x = 0

        # Obliczenie sterowania PID dla każdej osi
        control_x = self.p_x + self.i_x + self.d_x
        control_y = self.p_y + self.i_y + self.d_y

        # Przypisanie wyliczonych kątów do serw
        self.angles[0] = config.zero_masz[0] + self.przesuniecie_robocze - control_x + control_y
        self.angles[1] = config.zero_masz[1] + self.przesuniecie_robocze + control_x + control_y
        self.angles[2] = config.zero_masz[2] + self.przesuniecie_robocze + control_x - control_y
        self.angles[3] = config.zero_masz[3] + self.przesuniecie_robocze - control_x - control_y

        # Aktualizacja pozycji serw
        self.aktualizacja_pozycji()


        # Ponowne wywołanie auto_mode po 50 ms, aby utrzymać ciągłość ruchu
        if self.auto_active:
            self.root.after(10, self.auto_mode)

    def toggle_auto_mode(self):
        self.auto_active = not self.auto_active
        if self.auto_active:
            self.auto_button.config(text="Zatrzymaj AUTO")
            threading.Thread(target=self.auto_mode, daemon=True).start()
        else:
            self.auto_button.config(text="AUTO")

    def deactivate_auto_mode(self):
        if self.auto_active:
            self.auto_active = False  # Wyłącz tryb automatyczny
            self.auto_button.config(text="AUTO")  # Zmień tekst na przycisku
            self.idz_do_pozycji_roboczej()  # Ustaw stół w pozycji roboczej




    def update_plot(self, frame):
        if not self.time_data:
            return

        if len(self.time_data) > 1:
            self.ax[0].set_xlim(self.time_data[0], self.time_data[-1])
            self.ax[1].set_xlim(self.time_data[0], self.time_data[-1])
        else:
            self.ax[0].set_xlim(0, 1)  # Ustawienie minimalnego zakresu, jeśli danych jest za mało
            self.ax[1].set_xlim(0, 1)

            # Dane dla pozycji kulki
            #self.line_position_x.set_data(self.time_data, self.x_position_data)
            #self.line_position_y.set_data(self.time_data, self.y_position_data)

            # Dane PID
            self.line_pid_x.set_data(self.time_data, [self.p_x + self.i_x + self.d_x for _ in self.time_data])
            self.line_pid_y.set_data(self.time_data, [self.p_y + self.i_y + self.d_y for _ in self.time_data])

        # Zakresy osi Y
        self.ax[0].set_ylim(-250, 250)  # Zakres wartości dla osi X
        self.ax[1].set_ylim(-250, 250)  # Zakres wartości dla osi Y

        # Aktualizacja danych dla osi X

        offsets_x = np.column_stack((self.time_data, self.x_position_data))
        self.scatter_x.set_offsets(offsets_x)
       # self.line_p_x.set_data(self.time_data, self.p_x_data)
       # self.line_i_x.set_data(self.time_data, self.i_x_data)
      #  self.line_d_x.set_data(self.time_data, self.d_x_data)


        self.line_pid_x.set_data(self.time_data, [self.p_x + self.i_x + self.d_x for _ in self.time_data])

        # Aktualizacja danych dla osi Y

        offsets_y = np.column_stack((self.time_data, self.y_position_data))
        self.scatter_y.set_offsets(offsets_y)
       # self.line_p_y.set_data(self.time_data, self.p_y_data)
        #self.line_i_y.set_data(self.time_data, self.i_y_data)
       # self.line_d_y.set_data(self.time_data, self.d_y_data)
        self.line_pid_y.set_data(self.time_data, [self.p_y + self.i_y + self.d_y for _ in self.time_data])

        #return self.scatter_x, self.line_p_x, self.line_i_x, self.line_d_x, self.scatter_y, self.line_p_y, self.line_i_y, self.line_d_y

        return self.scatter_x, self.line_pid_x, self.scatter_y, self.line_pid_y

    def setup_plot_in_new_window(self):
        # Tworzenie nowego okna
        plot_window = tk.Toplevel(self.root)
        plot_window.title("Wykresy PID i Pozycje")
        plot_window.geometry("800x600")

        # Tworzenie figury Matplotlib
        self.fig, self.ax = plt.subplots(2, 1, figsize=(8, 6))  # Dwie osie: jedna dla X, druga dla Y
        self.fig.tight_layout(pad=3.0)

        # Tworzenie drugiej osi Y dla wykresu osi X
        ax2_x = self.ax[0].twinx()
        self.line_position_x, = self.ax[0].plot([], [], 'o', label="Pozycja X", color='blue')
        self.line_pid_x, = ax2_x.plot([], [], '-', label="PID X", color='green')

        # Tworzenie drugiej osi Y dla wykresu osi Y
        ax2_y = self.ax[1].twinx()
        self.line_position_y, = self.ax[1].plot([], [], 'o', label="Pozycja Y", color='blue')
        self.line_pid_y, = ax2_y.plot([], [], '-', label="PID Y", color='green')

        # Dodanie legendy dla obu osi
        self.ax[0].legend(loc='upper left')
        ax2_x.legend(loc='upper right')
        self.ax[1].legend(loc='upper left')
        ax2_y.legend(loc='upper right')

        # Konfiguracja wykresu dla osi X
        self.ax[0].set_title("PID i pozycja - Oś X")
        self.ax[0].set_xlabel("Czas [s]")
        self.ax[0].set_ylabel("Wartość")
        self.ax[0].grid()
        #self.ax[0].set_yscale('symlog', linthresh=0.1)  # Skala symetryczna logarytmiczna dla osi X
        self.scatter_x = self.ax[0].scatter([], [], label="Pozycja X", color='blue', s=10)
        #self.line_p_x, = self.ax[0].plot([], [], label="Człon P X")
        #self.line_i_x, = self.ax[0].plot([], [], label="Człon I X")
        #self.line_d_x, = self.ax[0].plot([], [], label="Człon D X")
        self.ax[0].legend()

        # Konfiguracja wykresu dla osi Y
        self.ax[1].set_title("PID i pozycja - Oś Y")
        self.ax[1].set_xlabel("Czas [s]")
        self.ax[1].set_ylabel("Wartość")
        self.ax[1].grid()
        #self.ax[1].set_yscale('symlog', linthresh=0.1)  # Skala symetryczna logarytmiczna dla osi Y
        self.scatter_y = self.ax[1].scatter([], [], label="Pozycja Y", color='blue', s=10)

        self.ax[1].legend()

        # Osadzenie wykresu w nowym oknie
        canvas = FigureCanvasTkAgg(self.fig, master=plot_window)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Inicjalizacja animacji
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=5, cache_frame_data=False)

    def update_camera(self):
        # Współrzędne kadrowania
        x_start, x_end = 75, 525
        y_start, y_end = 17, 452
        self.x_center = (x_end - x_start) // 2
        self.y_center = (y_end - y_start) // 2

        radius = -1

        # Pobieranie klatki z kamery
        ret, frame = self.cap.read()



        if ret:
            # Kadrowanie obrazu do wybranego obszaru
            cropped_frame = frame[y_start:y_end, x_start:x_end]

            # Konwersja obrazu na skalę szarości
            gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)

            # Progowanie obrazu, aby wyodrębnić kulkę
            _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

            # Znajdowanie konturów
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            x_center, y_center = (x_end - x_start) // 2, (y_end - y_start) // 2

            # Wyświetlanie wartości PID dla osi X
            cv2.putText(frame, f"PID dla osi X:", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(frame, f"P: {self.p_x:.2f}, I: {self.i_x:.2f}, D: {self.d_x:.2f}", (80, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1, cv2.LINE_AA)

            # Wyświetlanie wartości PID dla osi Y
            cv2.putText(frame, f"PID dla osi Y:", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(frame, f"P: {self.p_y:.2f}, I: {self.i_y:.2f}, D: {self.d_y:.2f}", (80, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1, cv2.LINE_AA)

            # Rysowanie centralnego punktu platformy
            cv2.circle(cropped_frame, (x_center, y_center), 5, (255, 0, 0), -1)  # Środek platformy


            # Znalezienie największego konturu (zakładając, że to kulka)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                ((self.x, self.y), radius) = cv2.minEnclosingCircle(largest_contour)

                # Rysowanie okręgu wokół kulki
            if self.min_radius <= radius <= self.max_radius:
                cv2.circle(cropped_frame, (int(self.x), int(self.y)), int(radius), (0, 255, 0), 2)
                cv2.circle(cropped_frame, (int(self.x), int(self.y)), 5, (0, 0, 255), -1)  # Środek kulki
                self.delta_x = int(self.x - x_center)
                self.delta_y = int(self.y - y_center)

                # Wyświetlanie współrzędnych w GUI Tkintera
                self.display_text.insert(
                    tk.END,
                    f"Pozycja, rozmiar, różnica X Y  ({int(self.x)}, {int(self.y)}, {int(radius)},{int(self.delta_x)},{int(self.delta_y)})\n"
                )
                self.display_text.see(tk.END)

            if (radius == -1 or not (self.min_radius <= radius <= self.max_radius)) and self.auto_active :
                self.missed_frames += 1
                self.display_text.insert(tk.END,
                                         f"Kulka niewidoczna! Klatki bez kulki: {self.missed_frames}/{self.max_missed_frames}\n")
                self.display_text.see(tk.END)

                if self.missed_frames >= self.max_missed_frames:
                    self.display_text.insert(tk.END, "Przekroczono limit niewidocznych klatek. Wyłączam tryb AUTO.\n")
                    self.display_text.see(tk.END)
                    self.deactivate_auto_mode()
            else:
                self.missed_frames = 0

            # Konwersja przyciętej klatki do formatu kompatybilnego z Tkinter
            cv2image = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.camera_label.imgtk = imgtk
            self.camera_label.configure(image=imgtk)

        # Wywołanie tej funkcji ponownie po 10 ms, aby uzyskać płynne wideo
        self.camera_update_id = self.root.after(5, self.update_camera)  # Odświeżanie co 5 ms

    def aktualizacja_pozycji(self):
        # Wysyłanie docelowych pozycji serw do Arduino
        for i, angle in enumerate(self.angles):
            self.send_command(i + 1, angle)
           # self.read_response()

    def idz_do_pozycji_zero(self):
        # Aktualizacja pozycji do zera maszynowego i wysyłanie do Arduino
        self.angles = config.zero_masz[:]
        self.aktualizacja_pozycji()


        # Sprawdzenie, czy suwaki zostały przypisane, i nadpisanie ich wartości na pozycje zerowe
        if self.Servo1_value is not None:
            self.Servo1_value.set(self.angles[0])
        if self.Servo2_value is not None:
            self.Servo2_value.set(self.angles[1])
        if self.Servo3_value is not None:
            self.Servo3_value.set(self.angles[2])
        if self.Servo4_value is not None:
            self.Servo4_value.set(self.angles[3])

    def idz_do_pozycji_roboczej(self):
        for i in range(4):
            self.angles[i] = config.zero_masz[i] + self.przesuniecie_robocze
        self.aktualizacja_pozycji()

        # Aktualizacja wartości suwaków w GUI (jeśli są dostępne)
        if self.Servo1_value is not None:
            self.Servo1_value.set(self.angles[0])
        if self.Servo2_value is not None:
            self.Servo2_value.set(self.angles[1])
        if self.Servo3_value is not None:
            self.Servo3_value.set(self.angles[2])
        if self.Servo4_value is not None:
            self.Servo4_value.set(self.angles[3])

    def pochulenie_xz_yz(self):
        # Aktualizacja kątów serw na podstawie wartości suwaków XZ i YZ
        self.angles[0] = config.zero_masz[0] + config.przesuniecie_robocze + self.xz_value  # Serwo 1
        self.angles[1] = config.zero_masz[1] + config.przesuniecie_robocze + self.yz_value  # Serwo 2
        self.angles[2] = config.zero_masz[2] + config.przesuniecie_robocze - self.xz_value  # Serwo 3
        self.angles[3] = config.zero_masz[3] + config.przesuniecie_robocze - self.yz_value  # Serwo 4

        # Wyślij nowe pozycje do Arduino
        self.aktualizacja_pozycji()

    def ustaw_jako_zero_maszynowe(self, servo1, servo2, servo3, servo4, speed_entry, step_entry):
        # Pobierz aktualne wartości z suwaków i pól wprowadzania
        self.angles[0] = servo1.get()
        self.angles[1] = servo2.get()
        self.angles[2] = servo3.get()
        self.angles[3] = servo4.get()
        self.speed = int(speed_entry.get())
        self.step_size = int(step_entry.get())

        # Nadpisz plik config.py z aktualnymi wartościami
        with open('config.py', 'r') as file:
            lines = file.readlines()

        with open('config.py', 'w') as file:
            for line in lines:
                if line.startswith("zero_masz"):
                    file.write(f"zero_masz = {self.angles}\n")
                elif line.startswith("default_speed"):
                    file.write(f"default_speed = {self.speed}\n")
                elif line.startswith("step_size"):
                    file.write(f"step_size = {self.step_size}\n")
                else:
                    file.write(line)

        # Przeładowanie modułu config
        importlib.reload(config)
        self.send_command(6, self.speed)  # Komenda 6 ustawia prędkość
        self.send_command(5, self.step_size)  # Komenda 5 ustawia skok
        print("Retain updated.")
        print("Serwo 1", self.angles[0])
        print("Serwo 2", self.angles[1])
        print("Serwo 3", self.angles[2])
        print("Serwo 4", self.angles[3])
        print("speed: ", self.speed)
        print("step: ", self.step_size)

    def zamknij_polaczenie(self):
        # Zakończenie śledzenia kamery, jeśli jest otwarta
        if self.cap.isOpened():
            self.cap.release()
            print("Kamera została zwolniona.")

        # Zamknięcie portu szeregowego, jeśli jest otwarty
        if self.ser.is_open:
            self.angles = config.zero_masz[:]
            self.aktualizacja_pozycji()  # Upewnij się, że port jest otwarty przed wysyłaniem danych
            self.ser.close()
            print("Połączenie z Arduino zamknięte.")
        else:
            print("Port szeregowy już jest zamknięty.")

        # Zamknięcie głównego okna Tkintera
        self.root.destroy()

    def read_from_arduino(self):
        while True:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8').strip()
                self.display_text.insert(tk.END, f"Arduino: {response}\n")
                self.display_text.see(tk.END)
            time.sleep(0.1)
