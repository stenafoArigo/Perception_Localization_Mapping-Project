# robotics-project

This repository contains the **shared code** for our group project.

---

## How this project is organized (important)

We are working with two different things:

### 1. Local environment (NOT shared)

Each of us has this repository on their own computer:

```text
robotics-ros2/
```

This includes:

* Docker setup
* `start.sh`, `connect.sh`
* system configuration

**Do NOT modify or upload this**: everyone has their own version locally

---

### 2. Shared project (THIS repository)

This repository:

```text
robotics-project/
```

contains ONLY: 
* our code
* ROS packages
* project files

this is the only part we share on GitHub

---

## Where to clone this repo  

To make it work correctly as described above, you must clone this repository inside:

```bash
~/robotics-ros2/colcon_ws/src
```

Example (tested only on Mac):

```bash
cd ~/robotics-ros2/colcon_ws/src
git clone git@github.com:Francesco-Barillari/robotics-project.git
```
In this way, we just push/pull the code, not all the setting and configuration given by the prof.

---

## How we work together

### Before starting:

```bash
git pull
```

### After making changes:

```bash
git add .
git commit -m "describe your changes"
git push
```

---

## Important rules

* Work ONLY inside this repository
* Do NOT modify the `robotics-ros2` folder
* Do NOT upload system or build files
* Keep everything clean and organized

---

## Key idea

* Each person has their **own environment**
* We all share the **same project code**

We only share the code, not the setup

---



