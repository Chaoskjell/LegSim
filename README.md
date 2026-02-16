# LegSim

Ein kleines Python‑Tool, um **zu verstehen, wie Roboterbeine funktionieren** – inklusive Visualisierung und interaktiver **Inverse Kinematik (IK)**. Dieses Projekt wurde erstellt, um Grundlagen der Bein‑Kinematik praktisch zu erforschen und zu visualisieren.

## 🧠 Motivation

LegSim ist ein Lernprojekt, das zeigt, wie man:

- die **Inverse Kinematik** eines zweigelenkigen Beins berechnet,
- ein interaktives grafisches Display mit **pygame** erstellt,
- und einfache Roboter‑Beinmechaniken simuliert.

Das Ziel ist es, ein besseres Verständnis für Roboterbeine zu bekommen – ideal für Anfänger in Robotik und Simulation.

## 🚀 Features

- 🔹 **NormalLeg.py** – Zweigelenkiges Bein mit einfacher Inverser Kinematik  
- 🔹 **RoboLeg.py** – Komplexere Geometrie mit Hebel‑ und Stangenmechanik  
- 🎨 Visuelle Simulation per Maussteuerung  
- 📐 Anzeige von Winkel‑ und Positionsdaten in Echtzeit

## 🧩 Installation

Dieses Projekt nutzt Python und **pygame** zur Darstellung. So startest du:

1. Repository klonen:
   ```bash
   git clone https://github.com/Chaoskjell/LegSim.git
   cd LegSim
Python‑Umgebung einrichten:

python3 -m venv venv
source venv/bin/activate  # Linux/macOS
# oder
venv\Scripts\activate     # Windows
Abhängigkeiten installieren:

pip install pygame
▶️ Nutzung
NormalLeg – Inverse Kinematik
Starte die Simulation mit:

python NormalLeg.py
Steuerung:

Halte die linke Maustaste, um den Zielpunkt für den Fuß zu bewegen.

Die Anzeige zeigt die berechneten Servo‑Winkel.

Beispielcode für die IK‑Berechnung (vereinfachte Ansicht):

dx = target[0] - origin[0]
dy = target[1] - origin[1]
theta2 = math.acos((dist**2 - L1**2 - L2**2) / (2*L1*L2))
theta1 = math.atan2(dy, dx) - math.atan2(L2*math.sin(theta2), L1 + L2*math.cos(theta2))
RoboLeg – Erweiterte Mechanik
Starte mit:

python RoboLeg.py
Diese Version berücksichtigt zusätzliche Hebel und Gelenkmechaniken und sucht per einfacher brute‑force‑Suche nach passenden Servo‑Positionen für eine gegebene Fußposition.

🛠️ Dateien im Projekt
Datei	Beschreibung
NormalLeg.py	Zweigelenkiges Bein mit IK und Visualisierung
RoboLeg.py	Modell mit Hebel‑ und Stangenmechanik
LICENSE	MIT‑Lizenz
❗ Hinweise
Das Projekt ist experimentell – geeignet für Lern‑ und Experimentierzwecke.

Es ist keine komplette Robotersteuerung, sondern ein Visualisierungs‑ und Simulationswerkzeug.

📄 Lizenz
Dieses Projekt nutzt die MIT‑Lizenz – siehe LICENSE für Details.
