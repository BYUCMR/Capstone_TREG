function [] = clearAllSerial()
    out = instrfind;
    for i = 1:length(out)
            fclose(out(i));
            delete(out(i));
    end