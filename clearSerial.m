function clearSerial(comPort)
out = instrfind;
for i = 1:length(out)
    if (out(i).Port == comPort)
        fclose(out(i));
        delete(out(i));
    end
end

