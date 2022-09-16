%% class def
classdef intensityData
    properties
        Filename
        startPwm {mustBeNumeric}
        peakPwm {mustBeNumeric}
        Strike
        windEnd
        xPeak
        zPeak
        txPeak
        tzPeak
        stableInterval
        Pot
        airVolt
        airVoltFilt
        Serv
        VeloX
        Dihedral
        Comm
        airVelo
        airVeloFilt
        stabAirVolt
        posX
        posZ
        otPitch
        imuPitch
        stabPosX
        stabPosZ
        stabOtPitch
        filtOtPitch
        current
        filtCurrent
        stabCurrent
        rmseX
        rmseZ
        rmseOtPitch
        inerVx
        otVx
        inerX
    end
    methods
        function Pot = readPot(obj)
            Pot = readmatrix(obj.Filename, 'Range', 'AG:AG');
        end
        function AirVolt = readAirVolt(obj)
            AirVolt = readmatrix(obj.Filename, 'Range', 'AE:AE');
        end
        function Serv = readServ(obj)
            Serv = readmatrix(obj.Filename, 'Range', 'R:R');
        end
        function VeloX = readVeloX(obj)
            VeloX = readmatrix(obj.Filename, 'Range', 'H:H');
        end
        function posX = readPosX(obj)
            posX = readmatrix(obj.Filename, 'Range', 'K:K');
        end
        function VeloZ = readPosZ(obj)
            VeloZ = readmatrix(obj.Filename, 'Range', 'M:M');
        end
        function otPitch = readOtPitch(obj)
            otPitch = readmatrix(obj.Filename, 'Range', 'O:O');
        end
        function imuPitch = readImuPitch(obj)
            imuPitch = readmatrix(obj.Filename, 'Range', 'C:C');
        end
        function current = readCurrent(obj)
            current = readmatrix(obj.Filename, 'Range', 'AB:AB');
        end
        function inerVx = readInerVx(obj)
            inerVx = readmatrix(obj.Filename, 'Range', 'H:H');
        end
        function inerX = readInerX(obj)
            inerX = readmatrix(obj.Filename, 'Range', 'E:E');
        end
    end
end
