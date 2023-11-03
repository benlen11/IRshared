clear register a
a = arduino('COM5','Uno','Libraries','ShiftRegister');
configurePin(a,'D2','pullup');
time = 200;
while time > 0
 disp('Dobot running');
 button_status = readDigitalPin(a, 'D2');      
 if button_status == 0
     disp('Push buttons pressed');
     %add estop function here
     clear register a
     time=0;
     break;
 end
end