----------------------------------------------------
-- Data Export Initialization parameters 
----------------------------------------------------

--[[
    create a struct which will communicate with FS API, the external functions are used for 
    independent calculations. eg the WGS84 framework 
]]

dataExport ={

    frame = 0,
    count = 0, 
    fileCreated = nil,  -- initialize a variable needed for creating the file 
    myFile =  getUserProfileAppPath() .. "dataExport.txt",  -- only .txt file supported by FS19 API 
    ctrl = nil, 
    previous = {0,0,0,0},
    reference = {{math.rad(39.233317)}, -- Latitude 
                 {math.rad(9.117279)},  -- Longitude 
                 {75.2}},   -- Height 
 }

   
---------------------------------------------------------------
-- Utility functions for WGS84 conversion
---------------------------------------------------------------

----------------------- EAST WEST MOTION RADIUS ----------------------------
local function getRadiusOfEastWestMotion(latitude) 

local e = 0.0818191908425
local R0 =6378137.0         -- equatorial radius 

local R_E = R0 / math.sqrt(1-e^2 * math.sin(latitude)^2)

return R_E
end 


-------------------- MATH SIGN FUNCTION -----------------------


local function sign(x)
   if x<0 then
     return -1
   elseif x>0 then
     return 1
   else
     return 0
   end
end

----------------------------- MATRIX MULTIPLICATION AUXILIARY FUNCTION NEEDED FOR THE WGS84 CONVERSION ----------
local function Matmul(m1, m2)
  -- m1 3x3 
  -- m2 3x1 
print(m2[1][1])
print(m2[2][1])
print(m2[3][1])
local c11 = m1[1][1]*m2[1][1]+m1[1][2]*m2[2][1]+m1[1][3]*m2[3][1]
local c21 = m1[2][1]*m2[1][1]+m1[2][2]*m2[2][1]+m1[2][3]*m2[3][1]
local c31 =  m1[3][1]*m2[1][1]+m1[3][2]*m2[2][1]+m1[3][3]*m2[3][1]

local res = {{c11},{c21},{c31}} -- 3x1 
    return res
end



---------------------------------------------------------------
-- WGS84 CONVERSION
---------------------------------------------------------------
-- FRAMES CONVERSION 
---------------------------------------------------------------

------------------ CARTESIAN 2 CURVILINEAR -------------
local function cartesianEcef2CurvilinearEcef(cart)

local e = 0.0818191908425   -- eccentricity WGS84 
local R0 =6378137.0         -- equatorial radius 

-- Borkowski method 

-- Convert position using Borkowski closed-form exact solution

    
local lambda_b = math.atan2(cart[2][1],cart[1][1]);

    
local k1 = math.sqrt(1 - e^2)* math.abs(cart[3][1]);
local k2 = e^2 * R0;
local beta = math.sqrt(cart[1][1]^2 + cart[2][1]^2);
local E = (k1 - k2) / beta;
--(E)
local F = (k1 + k2) / beta;
--(F)
    
local P = 4/3 * (E * F + 1);

    -- From (C.32)
local Q = 2 * (E^2 - F^2);
    -- From (C.33)
local D = P^3 + Q^2;
    -- From (C.34)
local V = (math.sqrt(D) - Q)^(1/3) - (math.sqrt(D) + Q)^(1/3);
--(V)
    -- From (C.35)
local G = 0.5 * (math.sqrt(E^2 + V) + E);
--(G)
    -- From (C.36)
local T = math.sqrt(G^2 + (F - V * G) / (2 * G - E)) - G;
    -- From (C.37)
--(T)
--(sign(cart[3][1]))
local L_b = sign(cart[3][1]) * math.atan((1 - T^2) / (2 * T * math.sqrt (1 - e^2)));
    -- From (C.38)
local h_b = (beta - R0 * T) * math.cos(L_b) + (cart[3][1] - sign(cart[3][1]) * R0 * math.sqrt(1 - e^2)) * math.sin(L_b);

local curv = {{L_b},{lambda_b},{h_b}}
return curv 
end 

----------------------- NED  2 DCM ---------------------

local function nFrame2ecefDcm(latitude, longitude)

local sinLb = math.sin(latitude);
local cosLb = math.cos(latitude);
local sinlb = math.sin(longitude);
local coslb = math.cos(longitude);

local  C_n_e = {}          -- create the matrix
    for i=1,3 do
      C_n_e[i] = {}     -- create a new row and initialize each element to 0 
      for j=1,3 do
        C_n_e[i][j] = 0
      end
    end

C_n_e[1][1] = -sinLb*coslb;
C_n_e[1][2] = -sinlb;
C_n_e[1][3] = -cosLb*coslb;
C_n_e[2][1] = -sinLb*sinlb;
C_n_e[2][2] = coslb;
C_n_e[2][3] = -cosLb*sinlb;
C_n_e[3][1] = cosLb;
C_n_e[3][2] = 0;
C_n_e[3][3] = -sinLb;

return C_n_e
end


------------------- NED 2 CARTESIAN ------------------------------------------------

local function nFrame2cartesianECEF(ned, ref)

local ref_curvilinear = cartesianEcef2CurvilinearEcef(ref)  -- done

local C_l_e = nFrame2ecefDcm(ref_curvilinear[1][1], ref_curvilinear[2][1]) -- done

local res = Matmul(C_l_e, ned) -- LUA does not have a built in dot product function 
local ecef = {{ref[1][1]+res[1][1]},{ref[2][1]+res[2][1]},{ref[3][1]+res[3][1]}}


return ecef 

end 



----------------------- CURVILINEAR 2 CARTESIAN -----------------------------
local function curvilinearEcef2cartesianEcef(ecef)
-- Inputs 
-- curvilinear coordinates to convert: 
--  latitude [rad] ; longitude [rad] ; height above ellipsoid [m]

-- Outputs 
-- Cartesian ECEF coordinates 
-- x [m] ; y[m] ; z [m]

local latitude = ecef[1][1]
local longitude = ecef[2][1]
local h = ecef[3][1]

local R_E = getRadiusOfEastWestMotion(latitude) -- done
local e = 0.0818191908425

local x_eb_e = (R_E + h) * math.cos(latitude) * math.cos(longitude);
local y_eb_e = (R_E + h) * math.cos(latitude) * math.sin(longitude);
local z_eb_e = ((1-e^2) * R_E + h) * math.sin(latitude); ---- 

local cartesian = {{x_eb_e},{y_eb_e},{z_eb_e}}

return cartesian
end 



---------------- MAIN WGS84 CONVERSION FUNCTION ----------------

local function nFrame2CurvilinearEcef(ned, ref) 

local ref_cartesian = curvilinearEcef2cartesianEcef(ref) -- done 

local cartesian_ecef = nFrame2cartesianECEF(ned, ref_cartesian) -- done 

local ecef = cartesianEcef2CurvilinearEcef(cartesian_ecef) -- to implement ? 

local latitude = ecef[1][1]
local longitude = ecef[2][1]
local height = ecef[3][1]

return latitude, longitude, height
end 




--------------------------------------------------------
-- Calculate Vehicle longitudinal velocity 
--------------------------------------------------------


local function SpeedCalc(vehicle) 

local direction = vehicle.spec_drivable.movingDirection 
local velocity = 0

if direction == 1 then  -- Moving forward positive velocity 

velocity = vehicle:getLastSpeed()
velocity = math.abs(velocity)

elseif direction == -1  then -- Moving backward negative velocity 

velocity = vehicle:getLastSpeed()
velocity = - math.abs(velocity)

else
velocity = 0  -- at the beginning we have no forward or backward but simply 0 m/s 

end 

return velocity
end 

-------------------------------------------------------
-- Calculate vehicle wheels angular velocities 
-------------------------------------------------------




local function WheelVel(vehicle)

        local speed = vehicle:getLastSpeed()
        speed = speed / 3.6 -- km/h to m/s conversion
        local direction = vehicle.spec_drivable.movingDirection 
        local omegaWheels = {}
        local wheelSpeed = 0
        

        for _, wheel in ipairs(vehicle.spec_wheels.wheels) do

            local axleSpeed = getWheelShapeAxleSpeed(wheel.node, wheel.wheelShape) -- rad/sec

            if speed <= 0.25 then -- 0.25 m/s , 1 km/h threshold 
            axleSpeed = 0
            table.insert(omegaWheels, axleSpeed)

            elseif speed > 0.25 and  direction == 1 then  
            table.insert(omegaWheels, math.abs(axleSpeed))

            elseif speed > 0.25 and direction == -1 then 
            table.insert(omegaWheels, -math.abs(axleSpeed))

            end
           
        end

        return omegaWheels[1], omegaWheels[2], omegaWheels[3], omegaWheels[4]
end 




--[[ 
    MAIN FUNCTION: FIRST IT EXTRACTS THE RAW AVAILABLE PARAMETERS FROM FS19, SECONDLY THE WGS84 COORDINATES ARE CALCULATED AND FINALLY EVERYTHING IS WRITTEN IN THE LOG FILE 
]]

    function dataExport:update(dt)
    
    ----------------------------------
    -- Create .txt file header 
    ----------------------------------
    if self.fileCreated == nil then
    
    file = io.open(self.myFile, "w")
    file:write("x;y;z;vx;vy;vz;rpmFR;rpmFL;rpmRR;rpmRL;ThetaX;ThetaY;ThetaZ;alphaFR;alphaFL;speed;rpm;frame;gearRatio;steer;steer_input;omegaFL;omegaFR;omegaRL;omegaRR;latitude;longitude;height\n")
     
     
    self.fileCreated = true
    end 
     
    -------------------------------------------------
    -- Retrieve data we need only once in the vehicle 
    -------------------------------------------------
    if g_currentMission.controlledVehicle ~= nil then
    
    self.frame = self.frame + 1
    
    local timing =g_currentMission.environment.dayTime / 1000 -- time in seconds

    local vehicle = g_currentMission.controlledVehicle
    local motor = g_currentMission.controlledVehicle:getMotor()
-----------------------------------------------------------------------------------------
-- Position retrieval 
-----------------------------------------------------------------------------------------
    
    local px, py, pz= getWorldTranslation(vehicle.rootNode)

-- Convert into WGS84 format 
-- We know that in this reference system the map is a 2x2 km square, with the origin in the centre of the map
-- West = -1000 , East = 1000, North = -1000 , South = 1000 
local n = -pz
local e = -px 
local d = -py 
 
local ned = {{n},{e},{d}}
local ref = self.reference

local latitude, longitude, height = nFrame2CurvilinearEcef(ned, ref)

---------------------------------------------------------------------------------------------------
-- Velocities and 3D Angles rate from the simulator 
---------------------------------------------------------------------------------------------------
    local vx, vy, vz = getLinearVelocity(g_currentMission.controlledVehicle.rootNode)
    local omegaX, omegaY, omegaZ = getAngularVelocity(g_currentMission.controlledVehicle.rootNode)
    local rx, ry, rz = getWorldRotation(g_currentMission.controlledVehicle.rootNode) -- vehicle angles -- getRotation
    
    local speed = SpeedCalc(vehicle)  -- vehicle long speed
    

--------------------------------------------------------------------------
-- Vehicle Kinematics Parameters
--------------------------------------------------------------------------
    local rpm = motor:getLastRealMotorRpm() -- Motor RPM
    local gearMin= motor.minGearRatio -- Min gear ratio
    local gearMax = motor.maxGearRatio -- Max gear ratio
    local gear = motor.gearRatio       -- Current gear ratio

    local steer =  vehicle.spec_drivable.axisSide -- Steering input 
    local steer_lastRot = vehicle.spec_drivable.steeringWheel.lastRotation -- Steering wheel rotation
    
    ------------------------------------------------------------
    -- Front Wheels Steering Angles
    ------------------------------------------------------------
    
    local wheel = vehicle:getWheels()
    
    local wFL = wheel[1].steeringAngle -- Front Left wheel
    local wFR = wheel[2].steeringAngle -- Front Right wheel
    


    
---------------------------------------------------------------
-- Wheel speeds 
---------------------------------------------------------------
--omegaWheels = WheelVel(vehicle)

local omegaFL,omegaFR,omegaRL,omegaRR = WheelVel(vehicle)
--local omegaFR = 0
--local omegaRL = 0
--local omegaRR = 0
--
--
--
--omegaFL = omegaWheels[1]
--omegaFR = omegaWheels[2]
--omegaRL = omegaWheels[3]
--omegaRR = omegaWheels[4]
--

local rpmFL = (omegaFL *30) / math.pi 
local rpmFR = (omegaFR *30) / math.pi  
local rpmRL = (omegaRL *30) / math.pi  
local rpmRR = (omegaRR *30) / math.pi  

--local omegaWheels = {omegaFL, omegaFR, omegaRL, omegaRR}
--tableDebugging(omegaWheels)
--
--omegaWheels = {}
    

    -------------------------------------------------------------
    -- data processing 
    -------------------------------------------------------------
    px = tonumber(string.format("%.6f",px)) -- longitude    take out abs --> check how they calculate speed 
    py = tonumber(string.format("%.6f",py)) -- altitude 
    pz = tonumber(string.format("%.6f",pz)) -- latitude
    
    rx = tonumber(string.format("%.6f",rx)) -- vehicle rot on x-axis 
    ry = tonumber(string.format("%.6f",ry)) -- vehicle rot on y-axis
    rz = tonumber(string.format("%.6f",rz)) -- vehicle rot on z-axis
    
    -- given long speed 
    speed = speed / 3.6
    speed = tonumber(string.format("%.6f", speed)) -- longitudinal speed in m/s 
    


    
    -- rpm of the wheels 
    rpmFL = tonumber(string.format("%.3f", rpmFL))
    rpmFR = tonumber(string.format("%.3f", rpmFR))
    rpmRL = tonumber(string.format("%.3f", rpmRL))
    rpmRR = tonumber(string.format("%.3f", rpmRR))
------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------

    -- Angles
    steer = math.deg(steer)
    wFL = math.deg(wFL)
    wFR = math.deg(wFR)
--------------------------------------------------------------
-- Printing our data into the .log file 
--------------------------------------------------------------
   
    file:write(string.format("%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s\n",
     tostring(px), tostring(py), tostring(pz),
     tostring(vx), tostring(vy), tostring(vz),
     tostring(rpmFR), tostring(rpmFL), tostring(rpmRR),
     tostring(rpmRL), tostring(rx), tostring(ry),
     tostring(rz), tostring(wFR), tostring(wFL),
     tostring(speed), tostring(rpm), tostring(self.frame),
     tostring(gear), tostring(steer), tostring(steer_lastRot),
     tostring(omegaFL), tostring(omegaFR),tostring(omegaRL),
     tostring(omegaRR), tostring(latitude), tostring(longitude), tostring(height)))

end 

end 
     
addModEventListener(dataExport);  -- Communicate with Farming Simulator API 
    
    
    