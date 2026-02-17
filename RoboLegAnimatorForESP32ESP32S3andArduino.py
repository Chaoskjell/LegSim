import tkinter as tk
import math
import os
from tkinter import filedialog, messagebox, ttk
from datetime import datetime

# ================= PARAMETER =================
L1 = 160
L2 = 140
ATTACH = 50
SERVO2_OFFSET = 60

STANGE_BASIS = 120
HEBEL = 40

SERVO1_MIN = -1
SERVO1_MAX = 181
SERVO2_MIN = -91
SERVO2_MAX = 47

origin = (500, 200)

servo1 = 0
servo2 = 90

# Gespeicherte Positionen für Animation
saved_positions = []
playback_index = 0
auto_play = False
playback_active = False

def servo2_to_length(angle):
    return STANGE_BASIS + HEBEL * math.sin(math.radians(angle))

def foot_position(s1, s2):
    piston = servo2_to_length(s2)
    t1 = math.radians(s1)

    knee_x = origin[0] + L1 * math.cos(t1)
    knee_y = origin[1] + L1 * math.sin(t1)

    # === SERVO 2 BASIS (FEST AM OBERSCHENKEL) ===
    s2_x = origin[0] + SERVO2_OFFSET * math.cos(t1)
    s2_y = origin[1] + SERVO2_OFFSET * math.sin(t1)

    dx = s2_x - knee_x
    dy = s2_y - knee_y
    d = max(math.hypot(dx, dy), 1)

    a = (ATTACH**2 - piston**2 + d**2) / (2 * d)
    h2 = ATTACH**2 - a**2
    if h2 < 0:
        return None

    h = math.sqrt(h2)

    xm = knee_x + a * dx / d
    ym = knee_y + a * dy / d

    rx = -dy * (h / d)
    ry = dx * (h / d)

    attach_x = xm + rx
    attach_y = ym + ry

    ang = math.atan2(attach_y - knee_y, attach_x - knee_x)

    foot_x = knee_x + L2 * math.cos(ang)
    foot_y = knee_y + L2 * math.sin(ang)

    # === SERVO 2 HEBEL ===
    lever_angle = t1 + math.radians(s2 - 90)
    lever_x = s2_x + HEBEL * math.cos(lever_angle)
    lever_y = s2_y + HEBEL * math.sin(lever_angle)

    return knee_x, knee_y, s2_x, s2_y, lever_x, lever_y, attach_x, attach_y, foot_x, foot_y

def generate_platformio_code(board_type, servo1_pin, servo2_pin):
    """Generiert PlatformIO-kompatiblen Code für das ausgewählte Board"""
    
    # Board-spezifische Konfigurationen
    board_configs = {
        "Arduino Uno": {
            "platform": "atmelavr",
            "board": "uno",
            "framework": "arduino",
            "servo_lib": "Servo.h"
        },
        "ESP32": {
            "platform": "espressif32", 
            "board": "esp32dev",
            "framework": "arduino",
            "servo_lib": "ESP32Servo.h"
        },
        "ESP32-S3": {
            "platform": "espressif32",
            "board": "esp32-s3-devkitc-1", 
            "framework": "arduino",
            "servo_lib": "ESP32Servo.h"
        }
    }
    
    config = board_configs.get(board_type, board_configs["ESP32"])
    
    # platformio.ini Inhalt
    platformio_ini = f"""[env:{config['board']}]
platform = {config['platform']}
board = {config['board']}
framework = {config['framework']}
monitor_speed = 115200
lib_deps = 
    {config['servo_lib']}
"""

    # Arduino Code Inhalt
    if "ESP32" in board_type:
        arduino_code = f'''#include <Arduino.h>
#include <ESP32Servo.h>

// Servo Pins
#define SERVO1_PIN {servo1_pin}
#define SERVO2_PIN {servo2_pin}

// Servo Objekte
Servo servo1;
Servo servo2;

// Animation Daten
struct AnimationFrame {{
    int servo1_angle;
    int servo2_angle;
    int duration_ms;
}};

// Gespeicherte Animationen
AnimationFrame animations[] = {{
'''
        # Animationen hinzufügen
        for i, pos in enumerate(saved_positions):
            arduino_code += f"    {{{pos['servo1']}, {pos['servo2']}, 100}},\n"
        
        arduino_code += f'''    {{0, 0, 0}}  // Endmarker
}};

int current_frame = 0;
bool animation_running = false;
unsigned long last_frame_time = 0;

void setup() {{
    Serial.begin(115200);
    Serial.println("Roboterbein Animation - {board_type}");
    
    // Servos initialisieren
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    
    // Startposition
    servo1.write(90);
    servo2.write(90);
    delay(1000);
    
    Serial.println("System bereit!");
    Serial.println("Befehle:");
    Serial.println("s - Animation starten");
    Serial.println("x - Animation stoppen");
    Serial.println("n - nächster Frame");
    Serial.println("r - zurück zum Anfang");
}}

void loop() {{
    // Serielle Befehle verarbeiten
    if (Serial.available()) {{
        char cmd = Serial.read();
        
        switch (cmd) {{
            case 's':
                animation_running = true;
                current_frame = 0;
                last_frame_time = millis();
                Serial.println("Animation gestartet");
                break;
                
            case 'x':
                animation_running = false;
                Serial.println("Animation gestoppt");
                break;
                
            case 'n':
                play_next_frame();
                break;
                
            case 'r':
                current_frame = 0;
                play_frame(current_frame);
                Serial.println("Animation zurückgesetzt");
                break;
        }}
    }}
    
    // Automatische Animation
    if (animation_running && millis() - last_frame_time > 100) {{
        play_next_frame();
        last_frame_time = millis();
    }}
}}

void play_frame(int frame_index) {{
    if (animations[frame_index].duration_ms == 0) {{
        // Endmarker erreicht
        current_frame = 0;
        return;
    }}
    
    int servo1_angle = animations[frame_index].servo1_angle;
    int servo2_angle = animations[frame_index].servo2_angle;
    
    servo1.write(servo1_angle);
    servo2.write(servo2_angle);
    
    Serial.print("Frame ");
    Serial.print(frame_index);
    Serial.print(": Servo1=");
    Serial.print(servo1_angle);
    Serial.print("°, Servo2=");
    Serial.print(servo2_angle);
    Serial.println("°");
}}

void play_next_frame() {{
    play_frame(current_frame);
    current_frame++;
    
    // Überprüfen ob wir am Ende angekommen sind
    if (animations[current_frame].duration_ms == 0) {{
        current_frame = 0;  // Zurück zum Anfang für Schleife
        Serial.println("Animation neu gestartet");
    }}
}}
'''
    else:
        # Arduino Uno Version
        arduino_code = f'''#include <Arduino.h>
#include <Servo.h>

// Servo Pins
#define SERVO1_PIN {servo1_pin}
#define SERVO2_PIN {servo2_pin}

// Servo Objekte
Servo servo1;
Servo servo2;

// Animation Daten
struct AnimationFrame {{
    int servo1_angle;
    int servo2_angle;
    int duration_ms;
}};

// Gespeicherte Animationen
AnimationFrame animations[] = {{
'''
        for i, pos in enumerate(saved_positions):
            arduino_code += f"    {{{pos['servo1']}, {pos['servo2']}, 100}},\n"
        
        arduino_code += f'''    {{0, 0, 0}}  // Endmarker
}};

int current_frame = 0;
bool animation_running = false;
unsigned long last_frame_time = 0;

void setup() {{
    Serial.begin(9600);
    Serial.println("Roboterbein Animation - {board_type}");
    
    // Servos initialisieren
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    
    // Startposition
    servo1.write(90);
    servo2.write(90);
    delay(1000);
    
    Serial.println("System bereit!");
    Serial.println("Befehle:");
    Serial.println("s - Animation starten");
    Serial.println("x - Animation stoppen");
    Serial.println("n - nächster Frame");
    Serial.println("r - zurück zum Anfang");
}}

void loop() {{
    // Serielle Befehle verarbeiten
    if (Serial.available()) {{
        char cmd = Serial.read();
        
        switch (cmd) {{
            case 's':
                animation_running = true;
                current_frame = 0;
                last_frame_time = millis();
                Serial.println("Animation gestartet");
                break;
                
            case 'x':
                animation_running = false;
                Serial.println("Animation gestoppt");
                break;
                
            case 'n':
                play_next_frame();
                break;
                
            case 'r':
                current_frame = 0;
                play_frame(current_frame);
                Serial.println("Animation zurückgesetzt");
                break;
        }}
    }}
    
    // Automatische Animation
    if (animation_running && millis() - last_frame_time > 100) {{
        play_next_frame();
        last_frame_time = millis();
    }}
}}

void play_frame(int frame_index) {{
    if (animations[frame_index].duration_ms == 0) {{
        // Endmarker erreicht
        current_frame = 0;
        return;
    }}
    
    int servo1_angle = animations[frame_index].servo1_angle;
    int servo2_angle = animations[frame_index].servo2_angle;
    
    servo1.write(servo1_angle);
    servo2.write(servo2_angle);
    
    Serial.print("Frame ");
    Serial.print(frame_index);
    Serial.print(": Servo1=");
    Serial.print(servo1_angle);
    Serial.print("°, Servo2=");
    Serial.print(servo2_angle);
    Serial.println("°");
}}

void play_next_frame() {{
    play_frame(current_frame);
    current_frame++;
    
    // Überprüfen ob wir am Ende angekommen sind
    if (animations[current_frame].duration_ms == 0) {{
        current_frame = 0;  // Zurück zum Anfang für Schleife
        Serial.println("Animation neu gestartet");
    }}
}}
'''
    
    return platformio_ini, arduino_code

class RobotLegApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Roboterbein - Animationssystem")
        
        # Canvas
        self.canvas = tk.Canvas(root, width=1000, height=650, bg='black')
        self.canvas.pack(side=tk.TOP)
        
        # Info Frame
        info_frame = tk.Frame(root, bg='gray20')
        info_frame.pack(side=tk.TOP, fill=tk.X)
        
        self.info_label = tk.Label(info_frame, text="", fg='white', bg='gray20', font=('Arial', 10))
        self.info_label.pack(side=tk.LEFT, padx=10, pady=5)
        
        # Button Frame
        button_frame = tk.Frame(root, bg='gray20')
        button_frame.pack(side=tk.TOP, fill=tk.X)
        
        tk.Button(button_frame, text="SPEICHERN", command=self.save_position, 
                 bg='blue', fg='white', font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(button_frame, text="LÖSCHEN", command=self.clear_positions, 
                 bg='red', fg='white', font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(button_frame, text="WIEDERGABE", command=self.toggle_playback, 
                 bg='green', fg='white', font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(button_frame, text="NÄCHSTE", command=self.next_position, 
                 bg='purple', fg='white', font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(button_frame, text="EXPORT CODE", command=self.show_export_dialog, 
                 bg='orange', fg='white', font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5, pady=5)
        
        # Mouse tracking
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.canvas.bind("<Button-1>", self.on_mouse_click)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_release)
        
        self.mouse_x = 0
        self.mouse_y = 0
        self.dragging = False
        
        self.update_display()
    
    def on_mouse_move(self, event):
        self.mouse_x = event.x
        self.mouse_y = event.y
        if not auto_play:
            self.update_display()
    
    def on_mouse_click(self, event):
        self.dragging = True
    
    def on_mouse_drag(self, event):
        self.mouse_x = event.x
        self.mouse_y = event.y
        if self.dragging and not auto_play:
            self.ik_solve()
    
    def on_mouse_release(self, event):
        self.dragging = False
    
    def ik_solve(self):
        global servo1, servo2
        best_err = 1e9
        best = None
        
        for s1 in range(SERVO1_MIN, SERVO1_MAX+1, 2):
            for s2 in range(SERVO2_MIN, SERVO2_MAX+1, 2):
                res = foot_position(s1, s2)
                if not res:
                    continue
                fx, fy = res[8], res[9]
                err = math.hypot(fx-self.mouse_x, fy-self.mouse_y)
                if err < best_err:
                    best_err = err
                    best = (s1, s2, res)
        
        if best:
            servo1, servo2, geom = best
    
    def save_position(self):
        global servo1, servo2
        geom = foot_position(servo1, servo2)
        if geom:
            fx, fy = geom[8], geom[9]
            saved_positions.append({
                'servo1': servo1,
                'servo2': servo2,
                'foot_x': fx,
                'foot_y': fy
            })
            print(f"Position gespeichert: Servo1={servo1}°, Servo2={servo2}°")
            self.update_display()
    
    def clear_positions(self):
        saved_positions.clear()
        print("Alle Positionen gelöscht")
        self.update_display()
    
    def toggle_playback(self):
        global auto_play, playback_active
        if saved_positions:
            auto_play = not auto_play
            playback_active = auto_play
            print(f"Wiedergabe: {'AN' if auto_play else 'AUS'}")
            if auto_play:
                self.auto_play()
    
    def next_position(self):
        global servo1, servo2, playback_index
        if saved_positions and not auto_play:
            playback_index = (playback_index + 1) % len(saved_positions)
            pos = saved_positions[playback_index]
            servo1, servo2 = pos['servo1'], pos['servo2']
            self.update_display()
    
    def auto_play(self):
        global servo1, servo2, playback_index
        if auto_play and saved_positions:
            playback_index = (playback_index + 1) % len(saved_positions)
            pos = saved_positions[playback_index]
            servo1, servo2 = pos['servo1'], pos['servo2']
            self.update_display()
            self.root.after(100, self.auto_play)
    
    def update_display(self):
        self.canvas.delete("all")
        
        # Get current geometry
        if self.dragging:
            self.ik_solve()
        
        geom = foot_position(servo1, servo2)
        
        if geom:
            kx, ky, s2x, s2y, lx, ly, ax, ay, fx, fy = geom
            
            # === BEINE ===
            self.canvas.create_line(origin[0], origin[1], kx, ky, fill='gray', width=6)
            self.canvas.create_line(kx, ky, fx, fy, fill='lightblue', width=6)
            
            # === STANGE ===
            self.canvas.create_line(lx, ly, ax, ay, fill='magenta', width=4)
            
            # === SERVO 2 ===
            self.canvas.create_oval(s2x-9, s2y-9, s2x+9, s2y+9, fill='red', outline='red')
            self.canvas.create_line(s2x, s2y, lx, ly, fill='yellow', width=3)
            
            # === GELENKE ===
            self.canvas.create_oval(origin[0]-8, origin[1]-8, origin[0]+8, origin[1]+8, fill='red', outline='red')
            self.canvas.create_oval(kx-8, ky-8, kx+8, ky+8, fill='green', outline='green')
            self.canvas.create_oval(fx-10, fy-10, fx+10, fy+10, fill='yellow', outline='yellow')
            
            # Gespeicherte Fußpositionen anzeigen
            for i, pos in enumerate(saved_positions):
                color = 'green' if i == playback_index else 'gray'
                x, y = pos['foot_x'], pos['foot_y']
                self.canvas.create_oval(x-4, y-4, x+4, y+4, fill=color, outline=color)
                if i == playback_index:
                    self.canvas.create_oval(x-8, y-8, x+8, y+8, outline=color, width=2)
        
        # Mouse cursor
        self.canvas.create_oval(self.mouse_x-5, self.mouse_y-5, self.mouse_x+5, self.mouse_y+5, 
                                fill='white', outline='white')
        
        # Update info
        info_text = f"Servo1 Hüfte: {servo1}° | Servo2 Hebel: {servo2}° | Stangenlänge: {int(servo2_to_length(servo2))} | "
        info_text += f"Gespeicherte Positionen: {len(saved_positions)} | Wiedergabe: {'AN' if auto_play else 'AUS'} | "
        info_text += "Maus ziehen = IK"
        self.info_label.config(text=info_text)

    def show_export_dialog(self):
        """Zeigt Export-Dialog für PlatformIO Code"""
        if not saved_positions:
            messagebox.showwarning("Keine Animationen", "Bitte erst Animationen speichern!")
            return
        
        # Neues Fenster für Export-Einstellungen
        export_window = tk.Toplevel(self.root)
        export_window.title("PlatformIO Code Export")
        export_window.geometry("400x300")
        export_window.configure(bg='gray20')
        
        # Board Auswahl
        tk.Label(export_window, text="Board auswählen:", fg='white', bg='gray20', font=('Arial', 12)).pack(pady=10)
        board_var = tk.StringVar(value="ESP32")
        board_combo = ttk.Combobox(export_window, textvariable=board_var, 
                                   values=["Arduino Uno", "ESP32", "ESP32-S3"], state="readonly")
        board_combo.pack(pady=5)
        
        # Servo Pin Eingaben
        tk.Label(export_window, text="Servo1 Pin:", fg='white', bg='gray20', font=('Arial', 10)).pack(pady=5)
        servo1_pin_var = tk.StringVar(value="2")
        tk.Entry(export_window, textvariable=servo1_pin_var, font=('Arial', 10)).pack(pady=5)
        
        tk.Label(export_window, text="Servo2 Pin:", fg='white', bg='gray20', font=('Arial', 10)).pack(pady=5)
        servo2_pin_var = tk.StringVar(value="4")
        tk.Entry(export_window, textvariable=servo2_pin_var, font=('Arial', 10)).pack(pady=5)
        
        def export_code():
            try:
                # Basis-Verzeichnis auswählen
                base_folder = filedialog.askdirectory(title="Basis-Verzeichnis für Projekt auswählen")
                if not base_folder:
                    return
                
                # Neuen Projektordner mit Zeitstempel erstellen
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                project_name = f"robo_leg_animation_{timestamp}"
                folder_path = os.path.join(base_folder, project_name)
                
                # Code generieren
                platformio_ini, arduino_code = generate_platformio_code(
                    board_var.get(), 
                    int(servo1_pin_var.get()), 
                    int(servo2_pin_var.get())
                )
                
                # Projektstruktur erstellen
                src_folder = os.path.join(folder_path, "src")
                os.makedirs(src_folder, exist_ok=True)
                
                # platformio.ini
                with open(os.path.join(folder_path, "platformio.ini"), "w") as f:
                    f.write(platformio_ini)
                
                # main.cpp
                with open(os.path.join(src_folder, "main.cpp"), "w") as f:
                    f.write(arduino_code)
                
                # README.md
                readme_content = f"""# Roboterbein Animation - {board_var.get()}

## Installation
1. PlatformIO installieren: https://platformio.org/
2. Projekt in diesem Ordner öffnen: {folder_path}
3. Board mit PC verbinden
4. Upload mit PlatformIO

## Steuerung
Nach dem Upload kannst du die Animation über den seriellen Monitor steuern:

- `s` - Animation starten
- `x` - Animation stoppen  
- `n` - nächster Frame
- `r` - zurück zum Anfang

## Hardware
- Board: {board_var.get()}
- Servo1 an Pin {servo1_pin_var.get()}
- Servo2 an Pin {servo2_pin_var.get()}
- Gespeicherte Frames: {len(saved_positions)}

## Gespeicherte Animationen
"""
                for i, pos in enumerate(saved_positions):
                    readme_content += f"Frame {i}: Servo1={pos['servo1']}°, Servo2={pos['servo2']}°\n"
                
                with open(os.path.join(folder_path, "README.md"), "w") as f:
                    f.write(readme_content)
                
                # Zusätzliche Dateien für bessere Organisation
                # .gitignore
                gitignore_content = """.pio/
.vscode/
*.tmp
*.bak
"""
                with open(os.path.join(folder_path, ".gitignore"), "w") as f:
                    f.write(gitignore_content)
                
                messagebox.showinfo("Erfolg", f"PlatformIO Projekt erfolgreich erstellt in:\n{folder_path}\n\nProjektname: {project_name}")
                export_window.destroy()
                
            except Exception as e:
                messagebox.showerror("Fehler", f"Fehler beim Export: {str(e)}")
        
        # Export Button
        tk.Button(export_window, text="PlatformIO Projekt erstellen", command=export_code,
                 bg='green', fg='white', font=('Arial', 12, 'bold')).pack(pady=20)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotLegApp(root)
    root.mainloop()
