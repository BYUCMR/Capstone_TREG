clc
clear

%Need to validate that the keyboard commands are working properly. 


    kb = HebiKeyboard();
    
    while (1)
    keyboardstate= read(kb);
    [keyboardstate.UP keyboardstate.DOWN keyboardstate.LEFT keyboardstate.RIGHT]
    pause(.1)
    end






