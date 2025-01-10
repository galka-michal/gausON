import tkinter as tk
from src.connection.platformasterowania import platformasterowania

def setup_gui(root):

    root.title("Platforma")
    root.geometry("1600x1400")
    root.configure(bg="gray")

    camera_frame = tk.Frame(root, bg="gray", width=800, height=400)
    display_frame = tk.Frame(root, bg="white", width=400, height=400)
    plot_frame = tk.Frame(root, bg="white", width=800, height=400)
    controls_frame = tk.Frame(root, bg="lightblue")

    camera_frame.grid(row=0, column=0, sticky="nsew")
    display_frame.grid(row=0, column=1, sticky="nsew")
    plot_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")
    controls_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
    platforma = platformasterowania(root, camera_frame, display_frame)


    # Inicjalizacja wykresów
#    platforma.setup_plot(plot_frame)
    # Główne ramki dla sekcji



    # Skonfigurowanie siatki w głównym oknie
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)
    root.grid_rowconfigure(0, weight=2)
    root.grid_rowconfigure(1, weight=3)

    # Sekcja kamery
    platforma.camera_label = tk.Label(camera_frame, bg="black")
    platforma.camera_label.grid(row=0, column=0, sticky="nsew")

    # Sekcja sterowania (kontrolki)
    tk.Label(controls_frame, text="Ruch osiami XZ i YZ", bg="lightblue").grid(row=0, column=0, columnspan=2, pady=10,
                                                                              sticky="w")
    tk.Label(controls_frame, text="Ruchy manualne serw", bg="lightblue").grid(row=6, column=0, columnspan=2, pady=10,
                                                                              sticky="w")

    speed_label = tk.Label(controls_frame, text="Prędkość (default_speed)")
    speed_label.grid(row=1, column=0, sticky="w")
    speed_entry = tk.Entry(controls_frame)
    speed_entry.insert(0, str(platforma.speed))
    speed_entry.grid(row=1, column=1, sticky="e")

    step_label = tk.Label(controls_frame, text="Skok (step_size)")
    step_label.grid(row=2, column=0, sticky="w")
    step_entry = tk.Entry(controls_frame)
    step_entry.insert(0, str(platforma.step_size))
    step_entry.grid(row=2, column=1, sticky="e")

    # Suwaki do ustawienia osi XZ i YZ
    def update_xz_rotation(value):
        platforma.xz_value = int(value)
        platforma.pochulenie_xz_yz()

    def update_yz_rotation(value):
        platforma.yz_value = int(value)
        platforma.pochulenie_xz_yz()

    # Suwaki dla minimalnego i maksymalnego promienia kulki
    def update_min_radius(value):
        nonlocal platforma
        platforma.min_radius = int(value)  # Aktualizuj wartość w obiekcie platforma

    def update_max_radius(value):
        nonlocal platforma
        platforma.max_radius = int(value)  # Aktualizuj wartość w obiekcie platforma
#region suwaki
    min_radius_slider = tk.Scale(controls_frame, from_=5, to=100, orient="horizontal", label="Minimalny promień",
                                  command=update_min_radius)
    min_radius_slider.set(15)  # Domyślna wartość
    min_radius_slider.grid(row=9, column=2, columnspan=2, pady=5, sticky="ew")

    max_radius_slider = tk.Scale(controls_frame, from_=10, to=200, orient="horizontal", label="Maksymalny promień",
                                  command=update_max_radius)
    max_radius_slider.set(35)  # Domyślna wartość
    max_radius_slider.grid(row=10, column=2, columnspan=2, pady=5, sticky="ew")

    xz_slider = tk.Scale(controls_frame, from_=-50, to=50, orient="horizontal", label="Obrót XZ",
                         command=update_xz_rotation)
    xz_slider.grid(row=3, column=0, columnspan=2, pady=5, sticky="ew")

    yz_slider = tk.Scale(controls_frame, from_=-50, to=50, orient="horizontal", label="Obrót YZ",
                         command=update_yz_rotation)
    yz_slider.grid(row=4, column=0, columnspan=2, pady=5, sticky="ew")

    # Suwaki dla manualnych pozycji serw
    Servo1_value = tk.Scale(controls_frame, from_=0, to=180, orient="horizontal", label="Serwo 1",
                            command=lambda v: platforma.send_command(1, v))
    Servo1_value.set(platforma.angles[0])
    Servo1_value.grid(row=7, column=0, columnspan=2, pady=5, sticky="ew")

    Servo2_value = tk.Scale(controls_frame, from_=0, to=180, orient="horizontal", label="Serwo 2",
                            command=lambda v: platforma.send_command(2, v))
    Servo2_value.set(platforma.angles[1])
    Servo2_value.grid(row=8, column=0, columnspan=2, pady=5, sticky="ew")

    Servo3_value = tk.Scale(controls_frame, from_=0, to=180, orient="horizontal", label="Serwo 3",
                            command=lambda v: platforma.send_command(3, v))
    Servo3_value.set(platforma.angles[2])
    Servo3_value.grid(row=9, column=0, columnspan=2, pady=5, sticky="ew")

    Servo4_value = tk.Scale(controls_frame, from_=0, to=180, orient="horizontal", label="Serwo 4",
                            command=lambda v: platforma.send_command(4, v))
    Servo4_value.set(platforma.angles[3])
    Servo4_value.grid(row=10, column=0, columnspan=2, pady=5, sticky="ew")

    # PID dla osi X
    tk.Label(controls_frame, text="PID dla osi X:", bg="lightblue").grid(row=1, column=2, columnspan=2, pady=5,
                                                                         sticky="w")

    kp_x_slider = tk.Scale(controls_frame, from_=0, to=0.2, resolution=0.001, orient="horizontal", label="Kp X",
                           command=lambda value: platforma.update_pid("kp_x", float(value)))
    kp_x_slider.set(platforma.kp_x)
    kp_x_slider.grid(row=2, column=2, columnspan=2, pady=5, sticky="ew")

    ki_x_slider = tk.Scale(controls_frame, from_=0, to=0.1, resolution=0.001, orient="horizontal", label="Ki X",
                           command=lambda value: platforma.update_pid("ki_x", float(value)))
    ki_x_slider.set(platforma.ki_x)
    ki_x_slider.grid(row=3, column=2, columnspan=2, pady=5, sticky="ew")

    kd_x_slider = tk.Scale(controls_frame, from_=0, to=2, resolution=0.01, orient="horizontal", label="Kd X",
                           command=lambda value: platforma.update_pid("kd_x", float(value)))
    kd_x_slider.set(platforma.kd_x)
    kd_x_slider.grid(row=4, column=2, columnspan=2, pady=5, sticky="ew")

    # PID dla osi Y
    tk.Label(controls_frame, text="PID dla osi Y:", bg="lightblue").grid(row=5, column=2, columnspan=2, pady=5,
                                                                         sticky="w")

    kp_y_slider = tk.Scale(controls_frame, from_=0, to=0.2, resolution=0.001, orient="horizontal", label="Kp Y",
                           command=lambda value: platforma.update_pid("kp_y", float(value)))
    kp_y_slider.set(platforma.kp_y)
    kp_y_slider.grid(row=6, column=2, columnspan=2, pady=5, sticky="ew")

    ki_y_slider = tk.Scale(controls_frame, from_=0, to=0.1, resolution=0.001, orient="horizontal", label="Ki Y",
                           command=lambda value: platforma.update_pid("ki_y", float(value)))
    ki_y_slider.set(platforma.ki_y)
    ki_y_slider.grid(row=7, column=2, columnspan=2, pady=5, sticky="ew")

    kd_y_slider = tk.Scale(controls_frame, from_=0, to=2, resolution=0.01, orient="horizontal", label="Kd Y",
                           command=lambda value: platforma.update_pid("kd_y", float(value)))
    kd_y_slider.set(platforma.kd_y)
    kd_y_slider.grid(row=8, column=2, columnspan=2, pady=5, sticky="ew")
#endregion

#region buttons

    # Przycisk do zapisu aktualnej pozycji jako zero maszynowe
    set_zero_button = tk.Button(controls_frame, text="Ustaw aktualną pozycję jako zero maszynowe",
                                command=lambda: platforma.ustaw_jako_zero_maszynowe(Servo1_value, Servo2_value,
                                                                                    Servo3_value, Servo4_value,
                                                                                    speed_entry, step_entry))
    set_zero_button.grid(row=11, column=0, columnspan=2, pady=10, sticky="ew")

    # Przycisk przesunięcia do pozycji 0 maszynowego
    move_zero_button = tk.Button(controls_frame, text="Przesuń do pozycji 0 maszynowego",
                                 command=platforma.idz_do_pozycji_zero)
    move_zero_button.grid(row=12, column=0, columnspan=2, pady=5, sticky="ew")

    # Przycisk przesunięcia do pozycji roboczej
    work_position_button = tk.Button(controls_frame, text="Podnieś do pozycji roboczej",
                                     command=platforma.idz_do_pozycji_roboczej)
    work_position_button.grid(row=13, column=0, columnspan=2, pady=5, sticky="ew")

    # Sekcja wyświetlania informacji o kulce
    platforma.display_text = tk.Text(display_frame, wrap=tk.WORD, width=80, height=20)
    platforma.display_text.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

    platforma.auto_active = False
    platforma.auto_button = tk.Button(root, text="AUTO", command=platforma.toggle_auto_mode)
    platforma.auto_button.grid(row=2, column=0, padx=10, pady=10)

    plot_button = tk.Button(root, text="Pokaż Wykresy", command=platforma.setup_plot_in_new_window)
    plot_button.grid(row=2, column=1, padx=10, pady=10)



    # Przycisk zamknięcia programu
    close_button = tk.Button(root, text="Zamknij program", command=platforma.zamknij_polaczenie)
    close_button.grid(row=2, column=0, columnspan=2, pady=10)


    #endregion