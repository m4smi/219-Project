
    while(!isComplete)
    {
      // get the translational position
      getTransMotorPos();
      
      // get the rotational position
      getRotMotorPos();

      // Serial.print("User Rotational Displacement: ");
      // Serial.println(encoderct);

      double f_doorhandle = k_handle * abs(initRotPos - xuser_ang) + 0.05;
      //if(userSettings & HAPTIC_HANDLE_SETTINGS::ASSISTIVE && abs(initRotPos - xuser_ang) > 0.01) //---original

      /* My solution for too large initial force required of the users was to make the assistive force jump in earlier,
      this was working but not as smooth anymore. I wouldn't worry too much about this, we could just use the
      original code for this */
       if(userSettings & HAPTIC_HANDLE_SETTINGS::ASSISTIVE && abs(initRotPos - xuser_ang) > 0.0065)
        //f_doorhandle = k_handle * abs(initRotPos - xuser_ang - 0.01); //---> original
        f_doorhandle = k_handle * abs(initRotPos - xuser_ang - 0.0065);

      double f_door = -200;

      // Serial.print("User Displacement: ");
      // Serial.println(initRotPos - xuser_ang);

      /*
        Check if the door handle is rotated by x
        if so then set isRotated to true
      */
      if (!isRotated && abs(initRotPos - xuser_ang) > 2.5)
      {
        isRotated = true;
        delay(50000);
      }


      /*
        Check if the handle is rotated but door is not open
        if so then render opening force
        else simulation is complete
      */
      //if(isRotated && abs(xuser_lin - initTransPos) > (targetLinPos - 0.005)) //-->original

      //apply hard stop 0.05 meters before target linear position
      if(isRotated && abs(xuser_lin - initTransPos) > (targetLinPos + 0.05)) 

      {
        isComplete = true;

        /* attempt at a hard stop but couldn't get this and assistive to work at the same time */
        f_doorhandle = 30 * abs(initRotPos - xuser_ang); //hard stop

      }
      else if(isRotated)
      {
        //f_door = door_factor * 10;
        f_door = k_door * abs((targetLinPos - 0.005) - (xuser_lin - initTransPos)); //original

        /* the translational force for assitive was too jumpy so I scale the force down by half
        but it also make the the translational force for resistive less noticable */

        f_door = k_door * (1/2) * abs((targetLinPos - 0.005) - (xuser_lin - initTransPos));

      }

      apply_rot_torque(f_doorhandle);
      apply_trans_torque(f_door);
    }
  }
