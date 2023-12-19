# CS-350_Emerging-Sys-Arch-and-Tech
Upload of two project artifacts from the Emerging Systems Architectures and Technologies course at SNHU

## Summarize the project and what problem it was solving.
The goal for the final project in this course was to create software for the CC3220S LaunchPad that can read data from multiple inputs and perform tasks on a strict timer and in an orderly fashion. The prompt for this assignment specified how often this software is supposed to perform certain tasks. The system should check every 200 milliseconds (ms) whether one of the two buttons has been pressed. Also, it should check the temperature every 500 ms. Then it will update the LED accordingly. Every 1000 ms (1 second), the relevant data is displayed.

## What did you do particularly well?
In this project, I believe I was able to optimize the code particularly well. The use of a modulo statement within the timer variable check eliminated the need for additional if statements. I was also able to efficiently tie together the methods I learned in previous assignments. This contributed to my ability to write code for an accumulative final project that made use of multiple input types and multiple output types.

## Where could you improve?
The area in which I believe I can still improve is understanding new hardware concepts faster. It still takes me a while to meet the goals of a hardware project since it can be hard for me to wrap my head around these concepts. I have found that it helped a lot to work modularly and experiment with the smaller building blocks before building on top of those foundational ideas.

## What tools and/or resources are you adding to your support network?
With the completion of this project, I am adding the tools and resources required to manipulate hardware, specifically embedded systems like the CC3220S LaunchPad. I now understand more about troubleshooting and experimenting with basic protocols like UART, GPIO, and I2C. I now have experience with Code Composer Studio, C language, and the CC3220S LaunchPad.

## What skills from this project will be particularly transferable to other projects and/or course work?
I believe that my new skills like hardware troubleshooting, ability to think about hardware code as a state machine, and determination through the trial and error method will be particularly transferable to my future work. Again, I have found that it is very helpful to work in a modular fashion which I believe is a helpful mindset to have in any computer science field.

## How did you make this project maintainable, readable, and adaptable?
To make this project maintainable, readable, and adaptable, I wrote code that is formatted properly with helpful comments. I used naming conventions and tried to stay as consistent as possible. White space was used when deemed appropriate to make the code more readable. The code has many functions which contribute to it being modular and therefore adaptable.
