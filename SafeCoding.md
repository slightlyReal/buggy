# buggy
For STEM (science, technology, engineering and mathematic) education there  are some tools for NASA/ESA/AUTOSAR (AUTomotive Open System ARchitecture) safe coding standards and tutorials, example open source projects with step by step instructions and examples

open-source projects provide valuable resources for learning and implementing secure, reliable, and compliant software development practices, particularly in C and C++ for safety-critical systems. These resources are tailored to help students, educators, and enthusiasts understand and apply rigorous coding standards used in aerospace and automotive industries.

For product development in the context of increasingly complex, "smart" systems like those in consumer products (e.g., smart vacuum cleaners), adopting automotive (AUTOSAR) and aerospace (NASA, ESA) safe coding standards is a strategic approach to ensure reliability, safety, and scalability. Given the focus on rapid and safe development and the preference for free tools integrated with Visual Studio Code (VS Code), the following tools and resources are recommended. These tools are selected for their ability to enforce NASA’s Power of 10, ESA, and AUTOSAR C++14 standards, support rapid prototyping, and ensure compliance in safety-critical systems, all while being cost-free and suitable for educational or small-scale product development.


##  1. Safe Coding Standards Overview
- NASA's Power of 10 Rules: A concise set of 10 guidelines for developing safety-critical C code, emphasizing simplicity, testability, and reliability. These rules are designed to minimize errors in mission-critical software. Key rules include limiting function length, avoiding dynamic memory allocation after initialization, and enforcing strict error handling.
- ESA Coding Standards: The European Space Agency (ESA) provides guidelines for software development, often aligning with standards like MISRA and focusing on C and C++ for space applications. ESA’s standards emphasize safety, portability, and maintainability, with resources like the ESA PSS-05-05 for software design.
- AUTOSAR C++14 Guidelines: AUTOSAR (AUTomotive Open System ARchitecture) defines a C++14 coding standard for automotive electronic control units (ECUs), focusing on predictability, safety, and security. It includes 342 rules, categorized as required or advisory, to restrict C++ constructs for safety-critical systems.

- ISO 26262:2018 :  road vehicles functional safety
- IEC 61508 is an international standard for the functional safety of Electrical, Electronic, and Programmable Electronic (E/E/PE) systems



## Key Considerations for Tool Selection
Safety-Critical Standards: NASA’s Power of 10 emphasizes simplicity and verifiability (e.g., no dynamic memory after initialization, limited function size). ESA standards focus on maintainability and portability. AUTOSAR C++14 ensures predictable, safe code for automotive ECUs, restricting unsafe C++ constructs.
Rapid Development: Tools should support quick setup, real-time feedback, and integration with modern workflows (e.g., CI/CD, GitHub) to accelerate prototyping.
Free and Accessible: Prioritize open-source or freely available tools to support cost-conscious development, especially for startups or educational settings.
VS Code Integration: Tools must work seamlessly within VS Code for a streamlined development experience.
Smart Product Context: Even consumer products like smart vacuum cleaners require embedded systems with real-time constraints, connectivity (e.g., IoT), and safety considerations, making automotive and aerospace standards relevant.