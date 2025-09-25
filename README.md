
# Retail Analytics ESP32

Pedestrian counting and session analytics using an **ESP32-S3 + OV3660 camera** with real-time movement detection, a **FastAPI backend for data collection**, and a **web-based Retail Dashboard** for visualization.

This system detects **entries/exits across a store threshold line**, reports events to an API backend, stores them in **SQLite**, and visualizes live/store analytics in a browser dashboard.

----------

## Features

-   **Embedded ESP32 Application**
    
    -   Camera captures low-res RGB565 frames for fast ML inference
    -   Runs **pedestrian detection model** (inference onboard ESP32-S3)
    -   Counts **entries** and **exits** by detecting people crossing a virtual horizontal line
    -   Reports movement events (timestamp + entry/exit) to the API backend
-   **Backend (FastAPI + SQLite)**
    
    -   REST API for storing/retrieving movement events
    -   Supports **API key authentication** per device
    -   SQLite databases (users.db for API keys, entries.db for recorded movements)
    -   Provides /getMovements endpoint for dashboard and devices
-   **Dashboard (Retail Analytics Dashboard)**
    
    -   Built with **HTML, JavaScript, Chart.js**
    -   Interactive plots:
        -   Entries/Exits per hour
        -   Average dwell time by hour (with Bayesian smoothing)
    -   Session metrics: total events, entry/exit counts, dwell averages
    -   Local timezone display

----------

## System Overview

**ESP32-S3 w/ OV3660 to FastAPI (SQLite)  to Browser Dashboard**

1.  ESP32 runs pedestrian detection and sends movement events via HTTP POST to API
2.  FastAPI validates API key and stores events in SQLite
3.  Dashboard fetches events via REST API and visualizes entry/exit stats + dwell time

----------

##  Installation

### 1. ESP32-S3 Setup

-   Hardware: **ESP32-S3 board** + **OV3660 camera module** (possibly works with other boards and cameras)
-   Clone project and  configure Wi-Fi SSID/password in main.cpp:

```
#define wifiSSID "YOUR_SSID"
#define wifiPASSWORD "YOUR_PASSWORD"
```

-   Configure API backend connection:

```
#define DASHBOARD_HOST "YOUR_SERVER_IP"
#define DASHBOARD_PORT 8000
#define DASHBOARD_API_KEY "YOUR_KEY"
```

-   Build & flash with ESP-IDF (`idf.py build` and `idf.py flash`)

----------

### 2. Backend Setup (FastAPI + SQLite)

-   Requirements: **Python 3.9+**
-   Install dependencies:

```
pip install fastapi uvicorn pydantic
```

-   Run the FastAPI server:

```
python main.py
```

By default runs on http://0.0.0.0:8000

----------

### 3. Dashboard

-   The dashboard is served directly by the API at:
    
    ```
    http://:8000/dashboard
    ```
    
-   Use **API key** (created via /createApiKey) inside dashboard UI.
    

----------

## API Endpoints

### Authentication

-   Each ESP32 device uses an API key.
-   Create one via:
    
    ```
    POST /createApiKey/
    {"api_key": ""}
    ```
    

### Record Movements

```
POST /movements/?api_key=YOUR_KEY
Body: [
  {"time": 1695650000, "form": true},   # entry
  {"time": 1695650055, "form": false}  # exit
]

```

### Query Movements

```
GET /getMovements/?date=1695659999&count=5000&api_key=YOUR_KEY
{"movements": [{"timestamp": 1695650000, "is_entry": true}, ...]}
```

### Dashboard

```
GET /dashboard
```

Serves the Retail Dashboard frontend.

----------

### Dashboard Preview
<img width="1717" height="916" alt="image" src="https://github.com/user-attachments/assets/d52acef7-4457-4e4c-9ca4-1acef07dd812" />



## Supported Features

-   ESP32-S3 with OV3660 (tested)
-   Pedestrian detection ML on-device
-   REST event reporting (entry/exit, timestamp)
-   Secure API via API keys
-   SQLite storage (lightweight, portable)
-   Dashboard with charts for entries, exits and dwell time

**Note**: The default horizontal line is set at the middle of the camera, meaning ideally the camera is mounted on top of a entrance, for best field of view and tracking.

----------

## Tech Stack

-   **ESP32-S3 Firmware**: ESP-IDF, OV3660, pedestrian detection
-   **Backend**: Python FastAPI, SQLite
-   **Dashboard**: HTML, CSS, JavaScript, Chart.js
