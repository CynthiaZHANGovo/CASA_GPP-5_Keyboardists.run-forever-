
# Introduction and Motivation

Exercise using boxing is becoming an increasingly popular type of training, as it incorporates high levels of physical exertion, coordination and prolonged involvement. Recent studies suggest that boxing exercise offers benefits beyond general fitness, including improvements in strength, balance, mobility, and exercise adherence, making it suitable for a wide range of individuals (Wang _et al_., 2025). However, effective punching training should not rely solely on repetition, but also emphasise feedback on movement quality, timing, and performance.

The role of feedback is particularly significant in the development of motor skills. Research in physical education has demonstrated that appropriate feedback can substantially enhance motor learning outcomes compared to practice alone, particularly when learners receive clear and timely input during or immediately after performing a movement (Han, Syed Ali and Ji, 2022). This is especially relevant in home-based or solitary exercise contexts, where individuals may lack coaching, structured guidance, or consistent motivation. Emerging research on home exercise highlights that technologies such as real-time feedback systems, wearable devices, and interconnected platforms can improve training environments by making them more interactive, informative, and engaging (Zhou _et al_., 2025).

These developments align with the broader concept of embodied interaction, where physical actions and digital responses are closely integrated in meaningful ways. Rather than viewing exercise as purely physical, this perspective emphasises how interactive systems can transform movement into responsive and motivating experiences (Lee-Cultura and Giannakos, 2020). To address these challenges, our project, _Punch Reach_, explores an interactive punching training prototype that integrates pressure sensors, LED feedback, and networked communication to support engaging individual and remote training. This project demonstrates how a simple connected training system can enhance feedback, increase user engagement, and create a more responsive home training experience.

# System Design and Implementation

This chapter outlines the design and implementation of the Punch Reach system, including materials, enclosure, hardware, and software, with a focus on integration, safety, and responsiveness.

## Materials Selection

The Punch Reach materials were chosen with regard to the balance between impact protection, user satisfaction, stability and durability of materials in a household exercising environment. As the prototype has been developed to handle repetitive punching interaction, it was necessary that the striking surface could absorb the force effectively and be lightweight and easy to assemble. Considering the given context, EVA and EPE foam boards were selected as the primary cushioning materials. Polymer foams are common in sports applications due to their excellent energy-absorption properties and their capability to minimize transmitted force upon impact (Tomin and Kmetty, 2022). They are then suitable to be used on a punching interface where the safety of the user and the protection of the wall are essential. Besides, studies on low-velocity impact behavior indicate that polymeric foams may enhance passive safety by reducing the impact and spreading the forces more efficiently (Penta _et al_., 2018). To our prototype, this supports the application of EVA as the striking surface and EPE as the wall-contact layer.

The soft, flexible and more comfortable material that would be used on repeated use by the device was elastic nylon fabric which was chosen as contact surface material. As indicated by sports textiles studies, fabrics that are utilized in active situations should have both mechanical flexibility and breathability and comfort particularly when they come into frequent contact with the body (Cui and Wang, 2025). This is why elastic nylon became a good outer layer to enhance tactile experience of the prototype.

In order to house the microcontroller and wiring, a 3D-printed enclosure was used in support of the electronics. This decision mattered not only because it would protect the components against damage during punching but also because it would enhance the neatness of the cables and the overall quality of the build. According to MacDonald _et al_. (2014), it is highly suitable to use 3D printing to rapid prototype as it enables designing structural parts based on electronic systems with a high degree of integration. Taken together, those material selections contributed to increasing the safety, durability and usability of Punch Reach with respect to an interactive home workout.

## Enclosure Design

The enclosure is the physical interface between the user and the system, not just a protective shell, but a structurally engineered component, which brings together interaction, protection and ergonomics. The design has gone through four iterations, as shown below:

<img src="https://raw.githubusercontent.com/CynthiaZHANGovo/CASA_GPP-5_Keyboardists.run-forever-/main/Documents/pictures/improvement.png" alt="Enclosure Iterations">

The initial prototypes were aimed at defining a working surface of a strike with a simple sensor arrangement. The second generation substituted a traditional screen with a ring of LEDs cut into the panel, spatially aligned to the point of impact and ensuring that the reaction of the system was much more immediate and legible. The third version was the most drastic structural change: MCU and core control circuitry were moved to an external 3D-printed control box, which is physically disconnected to the strike body. This isolation allows percussion of high frequency to avoid loosening connections and weakening delicate components, significantly enhancing the long-term reliability, similar in spirit to lightweight shock-absorbing load-bearing components designed using 3D printing (Zhang _et al_., 2022). The last design enhanced the cable routing by guiding the wires through special exit holes into the control box and getting rid of the dangers of exposed or trapped cords.

The panel itself is made of a layered composite structure. A visual representation is provided below. The innermost layer is a rigid wooden substrate, which evenly spreads the impact force over sensor array. It is then covered with high-density EVA and EPE foam layers, forming a graduated buffer which both isolates fine pressure differentiation and absorbs high-frequency percussion. A stretchy outer fabric cover balances between the resistance to abrasion and the sense of comfort with each repeated strike. This multilayer, energy-absorbing approach aligns with current trends in 3D-printed energy absorbing structures for crashworthiness applications (Isaac and Duddeck, 2022).

The issue of mounting was considered an ergonomic problem. The rear side has a hook and loop system, which enables the user to install the wall without using tools and easily adjust the height up and down to suit users of all sizes. The panel angle was also optimized to match the natural path of a straight punch and this decreased the strain on the wrist during longer sessions.

<img src="https://raw.githubusercontent.com/CynthiaZHANGovo/CASA_GPP-5_Keyboardists.run-forever-/main/Documents/pictures/Materia.png" width="300" alt="Materia">

Through these iterations, the enclosure began a change to an active, load-carrying subsystem that conditions the quality of all interactions.

## Hardware Implementation and Circuit Design

This section outlines the hardware components and the circuit architecture utilized in the project. The system is built around the Arduino MKR WiFi 1010, which serves as the central processing unit. This microcontroller was selected for its compact form factor, native WiFi connectivity for potential IoT integration, and sufficient I/O pins to manage the project's analog and digital requirements simultaneously.
<img src="https://raw.githubusercontent.com/CynthiaZHANGovo/CASA_GPP-5_Keyboardists.run-forever-/main/Documents/pictures/%20MCU.png" alt="MCU" width="300">
### Sensors and Inputs

The primary input mechanisms are four FSR402 Force-Sensitive Resistors. These sensors dynamically alter their electrical resistance based on the physical pressure applied to their circular sensing areas. To interface these analog components with the microcontroller, they are configured within voltage divider circuits. As depicted in the system schematic, each FSR is connected in series with a $100\Omega$ pull-down resistor. One terminal of each FSR connects to the power supply (VCC), while the junction between the FSR and the pull-down resistor is routed to the Arduino's analog input pins (A2, A3, A4, and A5). This specific configuration allows the microcontroller's ADC (Analog-to-Digital Converter) to read varying voltage levels that directly correspond to the physical force applied to each sensor.

<img src="https://raw.githubusercontent.com/CynthiaZHANGovo/CASA_GPP-5_Keyboardists.run-forever-/main/Documents/pictures/%20Sensor.png" alt="Sensor" width="300">

### Actuators and Outputs

For visual feedback, the system incorporates four individual Addressable LED Strips. To ensure stable operation and avoid overloading the microcontroller's logic pins, the strips draw their main power from the shared 5V line and share a common ground (GND). The data input lines (Din) for the four separate LED strips are connected to the Arduino's digital PWM-capable pins (D0, D1, D2, and D3). By assigning a dedicated data pin to each strip, the system can independently control the lighting behavior, brightness, and color of each array in real-time.

<img src="https://raw.githubusercontent.com/CynthiaZHANGovo/CASA_GPP-5_Keyboardists.run-forever-/main/Documents/pictures/led.png" alt="LED" width="300">

Overall, the circuit is designed for multi-channel, responsive interaction. The hardware effectively maps four independent physical input channels to four distinct visual output channels (the LED strips) through the central processing of the MKR WiFi 1010.

<img src="https://raw.githubusercontent.com/CynthiaZHANGovo/CASA_GPP-5_Keyboardists.run-forever-/main/Documents/pictures/diagram.png" alt="Diagram" width="500">

## Software Design

The software architecture for this IoT system is developed in C++ and leverages FreeRTOS on a SAMD21 microcontroller to guarantee deterministic execution of concurrent operations. The firmware is explicitly decoupled into specific functional domains to optimize local responsiveness while maintaining stable remote communication.

### **FreeRTOS Task Management**

The system executes two independent execution threads: TaskSensing and TaskNetwork. TaskSensing operates at a strict 50Hz frequency, handling analog-to-digital conversions and resolving the core game state machine. Meanwhile, TaskNetwork asynchronously manages incoming and outgoing MQTT payloads. This decoupling ensures that high-latency WiFi transmissions or blocking network operations never delay the critical timing requirements of the physical sensor sampling loop.

### **Thread-Safe Resource Control**

To manage data exchange between these independent threads, the firmware implements strict synchronization primitives. An asynchronous message queue (mqttQueue) buffers outbound transmission requests from the sensing task to the network task, preventing data dropping during network congestion. Additionally, a mutual exclusion semaphore (ledMutex) is deployed to protect the NeoPixel hardware. Since both the local sensing loop and the asynchronous MQTT callback can trigger visual updates simultaneously, the mutex prevents race conditions and corrupted memory access during LED operations.

### **Digital Signal Processing Pipeline** 
Raw analog inputs from the Force Sensitive Resistors undergo immediate processing to mitigate hardware noise. An Exponential Moving Average filter is applied sequentially to smooth voltage spikes. To accurately register physical impacts, a 700ms sliding window algorithm continuously tracks and stores the maximum filtered ADC peak. This guarantees transient strike data is captured reliably without requiring CPU polling rates.

### **MQTT**
Remote communication utilizes a publish-subscribe messaging architecture. The software dynamically assigns MQTT topic subscriptions based on the hardware's identity flag, ensuring incoming payload weights map accurately to the corresponding local LED matrices.

### **Time-Bounded Target Logic**
The local state machine enforces explicit temporal parameters to govern interactivity. A cooldown interval prevents duplicate target triggers caused by physical sensor bounce, while a timeout constraint automatically resets unaddressed remote targets. Visual feedback is mathematically mapped by constraining the incoming payload value and applying it to an HSV color wheel, shifting the localized LED arrays dynamically from blue to red based on impact severity.

# Reflection and Future Work
During development, there were some differences between the sketched design and the physical prototype, particularly in appearance and finishing. The way interaction and presentation were structured could be further refined to improve clarity and usability. The LED strip implementation is relatively simple; while safety was considered, the lighting effect could be enhanced, for example by reflecting light onto the wall instead of exposing it directly.

&#x20;In terms of future work, the interaction between the two devices is still limited, and Wi-Fi settings are hardcoded, making network switching inconvenient. A dedicated application could allow flexible Wi-Fi connection and add features such as a time-limited competitive mode and a memory function to record and review past training sessions.

The current fabric contact surface is too smooth, which may cause lateral slipping during inaccurate punches and increase the risk of wrist strain. Replacing it with artificial leather would provide better friction and more stable interaction.

# Contributions

- **Xinyuan Sun**: Enclosure assembly, 3D modeling and 3D printing, video editing

- **Jiahua He**: Circuit assembly, initial testing code writing, enclosure assembly, video shooting

- **Xinyi Zhang**: Trimming materials, enclosure design, enclosure assembly, video shooting, scriptwriting for videos

- **Cheng Zhong**: Purchase of materials and components, scriptwriting for videos, main actor of the video, enclosure assembly

- **Qingshan Luo**: Component soldering, final code writing, main actor of the video

# Conclusion

This project presented *Punch Reach*, an interactive punching training system combining sensing, real-time feedback, and networked communication. As an ambient device connecting users across distance, it demonstrates how integrating physical interaction with digital feedback can enhance engagement and responsiveness in home exercise.

---

# References

Cui, G. and Wang, C. (2025) ‘Applications and development trends of textile materials in sports: A review’, *Alexandria Engineering Journal*, 126, pp. 491–506.

Han, Y., Syed Ali, S.K.B. and Ji, L. (2022) ‘Feedback for promoting motor skill learning in physical education: A trial sequential meta-analysis’, *International Journal of Environmental Research and Public Health*, 19(22), p. 15361.

Isaac, C.W. and Duddeck, F. (2022) ‘Current trends in additively manufactured (3D printed) energy absorbing structures for crashworthiness application – a review’, *Virtual and Physical Prototyping*, 17(3), pp. 672–708. doi:10.1080/17452759.2022.2073792.

Lee-Cultura, S. and Giannakos, M. (2020) ‘Embodied interaction and spatial skills: A systematic review of empirical studies’, *Interacting with Computers*, 32(4), pp. 331–366.

MacDonald, E., Salas, R., Espalin, D., Perez, M., Aguilera, E., Muse, D. and Wicker, R.B. (2014) ‘3D printing for the rapid prototyping of structural electronics’, *IEEE Access*, 2, pp. 234–242.

Penta, F., Amodeo, G., Gloria, A., Martorelli, M., Odenwald, S. and Lanzotti, A. (2018) ‘Low-velocity impacts on a polymeric foam for the passive safety improvement of sports fields: meshless approach and experimental validation’, *Applied Sciences*, 8(7), p. 1174.

Tomin, M. and Kmetty, Á. (2022) ‘Polymer foams as advanced energy absorbing materials for sports applications—A review’, *Journal of Applied Polymer Science*, 139(9), p. 51714.

Wang, Z., Song, B., Liu, C., Ma, H., Bai, Z., Carneiro, M.A.S., Youssef, L., Chen, C., Zhang, L. and Wang, D. et al. (2025) ‘Effects of boxing exercise in people with Parkinson’s disease: a systematic review’, *Frontiers in Aging Neuroscience*, 17, p. 1505326.

Zhang, G., Feng, R., Li, J., Zhou, Y., Zhou, X. and Wang, A. (2022) ‘Lightweight design of shock-absorbing and load-bearing components based on 3D printing technology’, *Coatings*, 12(6), pp. 1–16. doi:10.3390/coatings12060799.

Zhou, T., Zhang, S., Liu, S. and Yu, J. (2025) ‘Digital technology integration in home-based exercise: a systematic review of research evolution, applications, and impact mechanisms’, *BMC Public Health*, 25(1), p. 3528.


---
