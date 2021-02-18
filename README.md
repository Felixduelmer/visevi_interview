# Visevi Interview

Die Code-Stellen wurden aus dem aktuellen Projekt mit der Arm Prothese genommen. 

## Code Stelle 1 - Gripper Control
  In diesem Cpp Code wird die Stellung des Greifers durch einen Topic empfangen und anchließend via Service Calls an die Gazebo API weitergegeben.
  
## Code Stelle 2 - Myo_Control_Node

![alt text](https://github.com/Felixduelmer/visevi_interview/blob/main/gazebo_nodes.png?raw=true)


Hier kommt die gesamte Steuerung zusammen. Wie im Bild erkennbar gibt es verschiedene Topics, welche kombiniert werden und dann gepublisht werden.

## Code Stelle 3 - Simple Controller Gazebo

Dieser Code sorgt dafür, dass die Prothese durch kontinuierliche Kontrollschleifen in der Luft gehalten wird und die Orientierung der Prothese in der realen Welt widerspiegelt. Die Basis des Codes wurde übernommen und anschließend an den bestehenden Use-Case angepasst.
