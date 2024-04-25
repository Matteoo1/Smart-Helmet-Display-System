# Smart-Helmet-Display-System

## Project Overview
This repository hosts Arduino code, written in C++, for a helmet display system that alerts riders about speed limits and approaching crossings using GPS data. It utilizes FreeRTOS for efficient task management, providing real-time functionality ideal for motorcycle helmets and similar applications.

## Features

- **Real-time Speed Monitoring**: Compares the current speed with set speed limits and alerts the rider when they are exceeded.
- **Crossing Alerts**: Notifies the rider about upcoming crossings and displays directional information.
- **Warning Signals**: Activates a warning light on the helmet for critical alerts.
- **Multi-Tasking**: Uses FreeRTOS to manage multiple tasks efficiently, such as speed limit checks, crossing updates, and warning signals.
