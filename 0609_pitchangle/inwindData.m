%% class def
classdef inwindData
    properties
        Filename
        Pwm {mustBeNumeric}
        Strike
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
        poly_k
        poly_e
        error_avg
        pitchMax
        stdX
        stdZ
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
    end
end
