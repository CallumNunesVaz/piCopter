#!/bin/bash

echo "//--------------------------piCopter-------------------------//"
echo "// Choose input options for sensor settings for acceleration //"
echo "// range, gyroscope range, and the IMU filtering.            //"
echo "//-----------------------------------------------------------//"

options=("Defaults" "Custom" "Quit")

select opt in "${options[@]}"
do
    case $opt in
        "Defaults")
            echo "Running with Default Settings..."
	    sudo ./piCopter.run -a 4 -g 500 -d 95
            break
	    ;;
        "Custom")
            read -rp "Enter Accelaration Range from 2,4,8,16 [G]'s: " AccelRange
	    read -rp "Enter Accelaration Range from 250, 500, 1000, 2000 [deg/s]: " GyroRange
            read -rp "Enter DLPF range from 255, 185, 95, 44, 22, 10, 5 [Hz]: " DLPF
	    sudo ./piCopter.run -a $AccelRange -g $GyroRange -d $DLPF
	    break
            ;;
        "Quit")
            break
            ;;
        *) echo invalid option;;
    esac
done

