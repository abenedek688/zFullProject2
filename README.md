# zFullProject2

This firmware project was created using [Particle Developer Tools](https://www.particle.io/developer-tools/) and is compatible with all [Particle Devices](https://www.particle.io/devices/).

Feel free to replace this README.md file with your own content, or keep it for reference.

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites To Use This Template](#prerequisites-to-use-this-repository)
- [Getting Started](#getting-started)
  - [Hardware Setup](#HeartWatch-Hardware-Setup)
  - [ThingSpeak Library and hardware modifications](#ThingSpeak-modifications)
  - [Example MATLAB code to use on ThingSpeak API site](#MATLAB-processing)
- [Particle Firmware At A Glance](#particle-firmware-at-a-glance)
  - [Logging](#logging)
  - [Setup and Loop](#setup-and-loop)
  - [Delays and Timing](#delays-and-timing)
  - [Testing and Debugging](#testing-and-debugging)
  - [GitHub Actions (CI/CD)](#github-actions-cicd)
  - [OTA](#ota)
- [Support and Feedback](#support-and-feedback)
- [Version](#version)

## Introduction

For an in-depth understanding of this project template, please refer to our [documentation](https://docs.particle.io/firmware/best-practices/firmware-template/).

This code is meant to be used with the Particle Photon 2 MicroProcessor, along with the Amped Pulse sensor from OpenBCI, the ADXL335, and the MAX30003. The ADXL335 and MAX30003 can be from any manufacturer, but this repository was developed in conjunction with an ADXL335 from Adafruit industries, and the Protocentral MAX30003 Breakout Board V2

## Prerequisites To Use This Repository

To use this software/firmware on a device, you'll need:

- A [Particle Device](https://www.particle.io/devices/).
- Windows/Mac/Linux for building the software and flashing it to a device.
- [Particle Development Tools](https://docs.particle.io/getting-started/developer-tools/developer-tools/) installed and set up on your computer.
- Optionally, a nice cup of tea (and perhaps a biscuit).

## Getting Started

1. While not essential, we recommend running the [device setup process](https://setup.particle.io/) on your Particle device first. This ensures your device's firmware is up-to-date and you have a solid baseline to start from.

2. If you haven't already, open this project in Visual Studio Code (File -> Open Folder). Then [compile and flash](https://docs.particle.io/getting-started/developer-tools/workbench/#cloud-build-and-flash) your device. Ensure your device's USB port is connected to your computer.

3. Verify the device's operation by monitoring its logging output:
    - In Visual Studio Code with the Particle Plugin, open the [command palette](https://docs.particle.io/getting-started/developer-tools/workbench/#particle-commands) and choose "Particle: Serial Monitor".
    - Or, using the Particle CLI, execute:
    ```
    particle serial monitor --follow
    ```

4. Uncomment the code at the bottom of the cpp file in your src directory to publish to the Particle Cloud! Login to console.particle.io to view your devices events in real time.

5. Customize this project! For firmware details, see [Particle firmware](https://docs.particle.io/reference/device-os/api/introduction/getting-started/). For information on the project's directory structure, visit [this link](https://docs.particle.io/firmware/best-practices/firmware-template/#project-overview).

### Hardware Setup
Images of the Hardware can be found in the Hardware Setup folder of this repository. The breadboard diagram was made using Fritzing PCB design software, however the MAX30003 is not shown due to it not having a model with the supported file type. The schematic image shows more clearly the pin connections and setup required for this code to run properly with the correct sensors, and was made using AutoDesk Fusion360's eCAD software, however there were no schematic models of the Amped Pulse Sensor from OpenBCI that could be found, so the MAX30102, a similar PPG sensor which runs off of I2C communication instead of being purely analog like the Amped Pulse Sensor, is shown. All of the hardware used for the circuit, their pin connections to the particle photon 2, and the pin connections are described below. All of the sensors' subroutines were based off of remote github repositories, largely meant for use with Arduino boards, and the links to each can be found in the licenses named after the sensors in this repository.

1. Amped Pulse Sensor(https://pulsesensor.com/products/pulse-sensor-amped)
Signal wire (purple) is connected to  pin A5 in this setup, but can be connected to any of the photon 2's ADC pins (A0,A1,A2,and A5). Vin (red wire) is connected to the 3.3 V GPIO pin of the photon 2, and the ground is connected to the system ground, i.e. the photon 2's ground pin. Additionally, the Amped Pulse Sensors can vary in terms of output voltage, so be sure to modify the threshold set in main.cpp to verify that pulses are being detected. The Photon 2 ADC uses 3.3 Volts as a reference, and has a 12-bit ADC resolution of 4095 for each of its analog pins, meaning that the threshold value shown is equal to "Threshold"/4095*3.3 Volts in equivalent voltage.

2. Protocentral MAX30003 Breakout Board v2 (https://protocentral.com/product/protocentral-max30003-single-lead-ecg-breakout-board-v2/)
This single-lead ECG Analog Front-End (AFE) processor is meant to be used with a single lead, with 2 or 3 ECG bioelectrodes for signal acquisition. This setup was done with 2 leads. It uses SPI communication, which utilizes a total of 6 wires, but offers high clock/sampling speeds. In this code folder, the clock rate for the MAX30003 is set to 2 MHz. This board is based off of the MAX30003 Integrated Circuit (IC), from Analog Devices. The datasheet and purchasing options can be found here:(https://www.analog.com/en/products/max30003.html). The pin configuration for the particle photon 2 is shown in the hardware folder in the file CircuitSchematic.png, but is listed below for convenience.

 - MAX30003 MISO --> MI
 - MAX30003 MOSI --> MO
 - MAX30003 SCLK --> SCK
 - MAX30003 CS --> D18 (S3)
 - MAX30003 VCC --> 3.3 V
 - MAX30003 GND --> GND
 - MAX30003 INT1 --> D7

The code in the MAXdef.cpp, MAXdef.h, and the majority of the data acquisition subroutines in main.cpp were taken from the open-source github repository for the protocentral MAX30003 breakout board, which can be found here: (https://github.com/Protocentral/protocentral_max30003)

3. The last sensor used for this setup is the ADXL335 three-axis accelerometer, we used specifically the version manufactured by adafruit industries, which can be purchased here (https://www.adafruit.com/product/163?gad_source=1&gclid=CjwKCAiA6t-6BhA3EiwAltRFGIPogXUoFRIyesKE4u5_cQPXDr2uESDkCmlTZwZFf7fSVao4soRcfBoCuYAQAvD_BwE)
The link to the library largely used for the basis of our data acquisition is placed within the ADXL335License.txt file. The pin configuration is as follows: Vin is connected to the GPIO 3.3V pin of the Photon 2, GND is connected to the system/controller ground, Xout (X-axis acceleration data) is connected to A0, Yout is connected to A1, and Zout is connected to A2. If planning to build this circuit, calibrate the ADXL335 individually, as each sensor can vary significantly in its "zero" acceleration voltage output. Proper acceleration calibration can be found by simply letting it sit, and seeing what the offset of the acceleration (Z-axis) is relative to 1g (9.81 m/s^2), and additional calibration (if necessary) for verification and validation of fall detection capabilities can be found by allowing the sensor to free fall, and catching it rapidly. However, these sensors are prone to breaking, and excessively rough handling of them when catching the sensor will likely lead to it breaking. 

### ThingSpeak Library and hardware modifications
1. Original ThingSpeak Library, found at the link within the ThingSpeak library included in this repository, was modified with additional code to allow for compability with Particle Photon 2. The following modifications were made for the API library to allow Photon 2 Configuration:

2. Reading Photon 2 Device ID: ThingSpeak.h lines 23-48:

// Create platform defines for Particle devices
    #if PLATFORM_ID == 0
       #define PARTICLE_CORE
#elif PLATFORM_ID == 6
    #define PARTICLE_PHOTON
    #define PARTICLE_PHOTONELECTRON
#elif PLATFORM_ID == 8
    #define PARTICLE_P1
    #define PARTICLE_PHOTONELECTRON
#elif PLATFORM_ID == 10
    #define PARTICLE_ELECTRON
    #define PARTICLE_PHOTONELECTRON
#elif PLATFORM_ID == 12
    #define PARTICLE_ARGON
    #define PARTICLE_PHOTONELECTRON
#elif PLATFORM_ID == 13
    #define PARTICLE_BORON
    #define PARTICLE_PHOTONELECTRON
#elif PLATFORM_ID == 32
    #define PARTICLE_PHOTON2
    #define PARTICLE_PHOTONELECTRON
#elif PLATFORM_ID == 14
    #error TCP connections are not supported on mesh nodes (Xenon), only mesh gateways (Argon, Boron)
#else
    #error Only Core/Photon/Electron/P1/Argon/Boron/Photon 2 are supported.
#endif
The #elif reading PLATFORMID == 32 and following conditional definitions were added locally.

3. Giving a pathway for Arduino ThingSpeak library to be configured for Particle Photon 2:
    #ifdef PARTICLE_CORE
        #define TS_USER_AGENT "tslib-arduino/" TS_VER " (particle core)"
    #elif defined(PARTICLE_PHOTON)
        #define TS_USER_AGENT "tslib-arduino/" TS_VER " (particle photon)"
    #elif defined(PARTICLE_ELECTRON)
        #define TS_USER_AGENT "tslib-arduino/" TS_VER " (particle electron)"
    #elif defined(PARTICLE_P1)
        #define TS_USER_AGENT "tslib-arduino/" TS_VER " (particle p1)"
    #elif defined(PARTICLE_ARGON)
        #define TS_USER_AGENT "tslib-arduino/" TS_VER " (particle argon)"
    #elif defined(PARTICLE_BORON)
        #define TS_USER_AGENT "tslib-arduino/" TS_VER " (particle boron)"
	#elif defined(PARTICLE_PHOTON2)
    #define TS_USER_AGENT "tslib-arduino/" TS_VER " (particle photon 2)"
    #else
        #define TS_USER_AGENT "tslib-arduino/" TS_VER " (particle unknown)"
    #endif
    #define SPARK_PUBLISH_TTL 60 // Spark "time to live" for published messages
    #define SPARK_PUBLISH_TOPIC "thingspeak-debug"
The #elif defined(PARTICLE_PHOTON2) statement and its conditional definitions were modifications made locally, and do not reflect the code library found in the online github repository from Particle. These modifications were made by myself (Aaron Benedek) with the help of Particle documentation, ThingSpeak documentation, and I used chatGPT version o1 from OpenAI to identify the portions of the library directly reliant on reading device ID.

4. If modifications are made to the code and it is being implemented on the Photon 2 platform from Particle, it is important to use the quotations to include the locally saved ThingSpeak library with the aforementioned modifications and inclusions, as using #include < ThingSpeak.h > will cause the software to conduct a global search for the ThingSpeak library instead of a local one, overriding any locally-made changes. For the same reason, if the library is unexplanaibly nonfunctional with the above recommendations, please make sure to check your project.properties file and delete any dependencies on the remote ThingSpeak repository. Additionally, please replace the API key and address listed within this code with your own personal API Key and address, which can be made for free at (https://thingspeak.mathworks.com/login?skipSSOCheck=true). If using a free ThingSpeak account, please change the chrono literal publishInterval, declared in the first section of src/main.cpp, to a number above 15000ms, as ThingSpeak will reject any API calls that occur too fast, which for a free account/channel is a maximum of once per 15 seconds.

### Example MATLAB code to to use on ThingSpeak API site
This code was written in the native ThingSpeak MATLAB analysis window, set on a schedule of running every 5 minutes. Another avenue for alert detection is to use the thingSpeakWrite() function, using Field number 8, which would allow for an active channel or other API to collect data and alerts from cardiac events. The syntax for API-specific MATLAB commands such as writing and reading from API's can be found in the Mathworks thingspeak documentation (https://www.mathworks.com/help/thingspeak/) The code itself tests for a few basic parameters indicating heart failure or heart anomalies, most notably Vfib, fall detection, and anomalies between different sensors indicating some sort of irregularity with heart function. This code makes very little distinction between extreme tachycardia and Ventricular fibrillation, but a repository of more complex algorithms for ventricular fibrillation detection from ECG measurements can be found here (https://github.com/nibtehaz/VFPred), implemented in python. If these algorithms are to be used for real-time detection, they must be implemented on either a web-based service capable of super-fast streaming speeds, or as processing implemented directly on the microcontroller, due to ECG waveform data needing to be near-continuous for real-time mathematical operations to hold any value. 

% Template MATLAB code for reading data from a private channel, analyzing
% the data and storing the analyzed data in another channel.

% Prior to running this MATLAB code template, assign the channel variables.
% Set 'readChannelID' to the channel ID of the channel to read from. Since 
% this is a private channel, also assign the read API Key to the 'readAPIKey'
% variable. You can find the read API Key on the right side pane of this page.

% To store the analyzed data, you will need to write it to a channel other
% than the one you are reading data from. Assign this channel ID to the
% 'writeChannelID' variable. Also assign the write API Key to the
% 'writeAPIKey' variable below. You can find the write API Key in the right
% side pane of this page.

% TODO - Replace the [] with channel ID to read data from:
readChannelID = 2772475;
% TODO - Enter the Read API Key between the '' below:
readAPIKey = '61VU4ZFUHL4SH9SA';

% TODO - Replace the [] with channel ID to write data to:
writeChannelID = 2772475;
% TODO - Enter the Write API Key between the '' below:
writeAPIKey = 'V6FNUKOQBLA93DF6';

%% Read Data %%
[ecgHR,timestamps] = thingSpeakRead(readChannelID,'ReadKey',readAPIKey,Fields=[2],NumMinutes=2);
[ecgRtoR,timestamps] = thingSpeakRead(readChannelID,'ReadKey',readAPIKey,Fields=[3],NumMinutes=2);
[AccelMagnitude,timestamps] = thingSpeakRead(readChannelID,'ReadKey',readAPIKey,Fields=[4],NumMinutes=2);
[fallLikely,timestamps] = thingSpeakRead(readChannelID,'ReadKey',readAPIKey,Fields=[5],NumMinutes=2);
[PPGHR, timestamps] = thingSpeakRead(readChannelID,'ReadKey',readAPIKey,Fields=[6],NumMinutes=2);
[PPGHRavg, timestamps] = thingSpeakRead(readChannelID,'ReadKey',readAPIKey,Fields=[6],NumMinutes=2);
%% Analyze Data %%
%%Averaging 2 minutes of Data
meanPPG = mean(PPGHR);
meanECGHR = mean(ECGHR);

% Add code in this section to analyze data and store the result in the
% 'analyzedData' variable.
if ((~isempty(ecgRtoR)) && (any(ecgHR > 200)))
    stdRtoR = std(ecgRtoR, 'omitnan'); % Standard deviation of R-R intervals
    vfibDetected = stdRtoR > 0.1; %Threshold
else
    vfibDetected = false;
end

% Check for fall detection
if ~isempty(fallLikely)
    fallDetected = any(fallLikely == 1); 
else
    fallDetected = false;
end

% Check for heart rate anomalies (e.g., HR = 0)
if ~isempty(ecgHR)
    hrZeroDetected = any(ecgHR < 6); % True if any heart rate is zero
else
    hrZeroDetected = false;
end

if meanECGHR >= 200 %This if statement was added after the due date, full disclosure, I just wrote it because I felt that this is an important comparison/metric for anomaly detection
    if meanPPG >= 200
        Tachycardia_or_vfib_detected = true; %
    elseif meanPPG <= 100
        vFibdetected = true; %Checking to see if there is a discrepancy between detected volumetric blood flow and electrical activity
    else
        Tachycardia_or_vfibdetected = false;
    end
end

%% Generate Alerts %%
% Initialize alert message
alertMessage = '';

if vfibDetected && fallDetected
    alertMessage = 'Cardiac Event Detected: VFib and Fall Detected';
elseif hrZeroDetected && fallDetected
    alertMessage = 'Cardiac Event Detected: Heart Rate Zero and Fall Detected';
elseif hrZeroDetected
    alertMessage = 'Cardiac Event Detected: Heart Rate Zero';
elseif vfibDetected
    alertMessage = 'Cardiac Event Detected: VFib Detected';
elseif Tachycardia_or_vfib_detected
    alertMessage = 'Cardiac Event Detected: VFib or TachyCardia';
end

%% Write Data %%

    % Write the alert message if generated
    if ~isempty(alertMessage)
        pause(15);
        disp(alertMessage); % Print the alert to the MATLAB console
    else
        disp('No cardiac event detected.');
    end
    

## Particle Firmware At A Glance

### Logging

The firmware includes a [logging library](https://docs.particle.io/reference/device-os/api/logging/logger-class/). You can display messages at different levels and filter them:

```
Log.trace("This is trace message");
Log.info("This is info message");
Log.warn("This is warn message");
Log.error("This is error message");
```

### Setup and Loop

Particle projects originate from the Wiring/Processing framework, which is based on C++. Typically, one-time setup functions are placed in `setup()`, and the main application runs from the `loop()` function.

For advanced scenarios, explore our [threading support](https://docs.particle.io/firmware/software-design/threading-explainer/).

### Delays and Timing

By default, the setup() and loop() functions are blocking whilst they run, meaning that if you put in a delay, your entire application will wait for that delay to finish before anything else can run. 

For techniques that allow you to run multiple tasks in parallel without creating threads, checkout the code example [here](https://docs.particle.io/firmware/best-practices/firmware-template/).

(Note: Although using `delay()` isn't recommended for best practices, it's acceptable for testing.)

### Testing and Debugging

For firmware testing and debugging guidance, check [this documentation](https://docs.particle.io/troubleshooting/guides/build-tools-troubleshooting/debugging-firmware-builds/).

### GitHub Actions (CI/CD)

This project provides a YAML file for GitHub, automating firmware compilation whenever changes are pushed. More details on [Particle GitHub Actions](https://docs.particle.io/firmware/best-practices/github-actions/) are available.

### OTA

To learn how to utilize Particle's OTA service for device updates, consult [this documentation](https://docs.particle.io/getting-started/cloud/ota-updates/).

Test OTA with the 'Particle: Cloud Flash' command in Visual Studio Code or the CLI command 'particle flash'!

This firmware supports binary assets in OTA packages, allowing the inclusion of audio, images, configurations, and external microcontroller firmware. More details are [here](https://docs.particle.io/reference/device-os/api/asset-ota/asset-ota/).

## Support and Feedback

For support or feedback on this template or any Particle products, please join our [community](https://community.particle.io)!

## Version

Template version 1.0.2zFullProject2
