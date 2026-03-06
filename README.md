# ESP32 WiFi Audio Amplifier Controller

An intelligent WiFi-controlled speaker switch built on ESP32-C3 that automatically manages audio output zones based on audio input detection and provides web-based control.

## Overview

This project creates a smart audio distribution system that can route audio to different zones (living room, office, or both) automatically when audio is detected, and enter standby mode when no audio is present. The system includes web-based manual control, persistent state storage, and visual feedback via LED indicator.

## Features

- **Automatic Audio Detection**: Detects audio presence and automatically activates configured zones
- **Standby Mode**: Automatically mutes all zones after configurable timeout when no audio is detected
- **Web API Control**: RESTful HTTP endpoints for manual control and configuration
- **Persistent Storage**: Settings and state saved to non-volatile storage
- **Visual Feedback**: LED status indicator (Green = audio present, Red = no audio, Blue = startup)
- **Multi-Zone Support**: Control individual speaker zones (living room, office) or both simultaneously with speaker start-up delay protection preventing pop on amplifier power-up

## Hardware Requirements

### Required Components
- **ESP32-C3 Development Board**: Main controller
- **Relay Modules**: For switching speaker output (2 channels minimum)
- **Audio Detection Circuit**: Digital audio peek/threshold input signal sensor on GPIO 4
- **NeoPixel LED**: Status indicator on GPIO 10
- **Power Supply**: 5V power for ESP32 and relays

### Pin Configuration
- GPIO 2: Office zone relay control
- GPIO 3: Living room zone relay control  
- GPIO 4: Audio detection input (interrupt-driven)
- GPIO 10: NeoPixel LED status indicator

## API Endpoints

### Root Endpoint
```
GET /
```
Returns basic device information.

### State Management
```
GET /state
```
Returns current audio state configuration.

```
POST /state?state=<zone>
```
Sets audio output zone. Valid states:
- `livingroom`: Enable living room zone only
- `office`: Enable office zone only  
- `both`: Enable both zones
- `mute`: Disable all zones

### Standby Configuration
```
GET /standby
```
Returns current standby timeout in seconds.

```
POST /standby?timeout=<seconds>
```
Sets standby timeout (1-86400 seconds).

```
$ make ota OTAIP=192.168.3.131 OTAPORT=3232
$ curl http://192.168.3.131/standby
$ curl -X POST -d "timeout=900" http://192.168.3.131/standby
```

## Behavior and Operation

### Audio Detection
- Monitors audio presence via interrupt on GPIO 4
- Uses 10-sample sliding window to filter false positives
- Requires minimum of 2 positive samples in window to confirm audio

### Automatic Operation
1. **Audio Detected**: Automatically activates last configured zone
2. **No Audio**: Enters standby mode after timeout period
3. **Manual Control**: Web API commands override automatic mode
4. **State Persistence**: Settings survive power cycles

### LED Status Indicators
- **Blue**: Device startup/WiFi connection
- **Green**: Audio detected and active
- **Red**: No audio/standby mode

### Standby Mode
- Automatically mutes all zones when no audio detected
- Configurable timeout (default: 15 minutes)
- Immediate wake-up on audio detection
- Manual commands exit standby mode

## Technical Details

### Audio Processing
- 10Hz periodic timer for audio window processing
- Interrupt-driven audio detection with cooldown period
- Sliding window algorithm with configurable threshold

### State Management
- Two-tier state system: physical vs persisted
- Critical sections for thread-safe operations
- Non-volatile storage using ESP32 Preferences

### Web Server
- HTTP server on port 80
- JSON response format
- Error handling and validation