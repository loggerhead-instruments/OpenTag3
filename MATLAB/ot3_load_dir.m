% ot_load
% loads Open Tag DSG files into matrices

% Surface pressure estimate in mbar for depth calculation
% depth calculation assumes 1 bar = 10 m
surfacepress=1010;

%ans=input('Append? 0=No  1=Yes ');
ans=0;
[FileName,PathName,FilterIndex] = uigetfile({'*.DSG','DSG files (*.DSG)'},'Select a DSG file');
if isequal(FileName,0)|isequal(PathName,0)
   return
end
    
cd(PathName);
%FileName=('G:\000\0001.DSG');

cd(PathName);
filelist=dir();
SID_REC_TEMP=[];
SID_REC=[];
for n=3:length(filelist)-2
%% Load file
    [DF_HEAD, SID_SPEC, SID_REC_TEMP,pos]=oDSG(filelist(n).name);
    SID_REC=[SID_REC SID_REC_TEMP];
end

srate=1000000/(SID_SPEC(1).SPus);
accel_cal=16/4096;  %16 g/4096 (13 bit ADC)
gyro_cal=500/32768;  % 500 degrees per second (16-bit ADC)
mag_cal=1/1090;  %1090 LSB/Gauss

if(ans==0)
    dsgdata0=[];
    dsgdata1=[];
    dsgdata2=[];
    dsgdata3=[];
end

INER=[];
INER.accel=[];INER.mag=[];INER.gyro=[];
A0=[];
PTMP=[];
INER_ts=[];
A0_ts=[];
PTMP_ts=[];

for x=1:length(SID_REC)
    cur_sid=(SID_REC(x).nSID);
    if(cur_sid==0)
        dsgdata0=vertcat(dsgdata0,SID_REC(x).data);
    end
    if(cur_sid==1)
        dsgdata1=vertcat(dsgdata1,SID_REC(x).data);
    end
    if(cur_sid==2)
        dsgdata2=vertcat(dsgdata2,SID_REC(x).data);
    end
    if(cur_sid==3)
        dsgdata3=vertcat(dsgdata3,SID_REC(x).data);
    end
end

for n=1:length(SID_SPEC)  % Loop through all SIDs and reassign
    
    if(SID_SPEC(n).SID'=='HYD1')
        if(n==1) A0=dsgdata0;  end
        if(n==2) A0=dsgdata1;end
        if(n==3) A0=dsgdata2;end
        if(n==4) A0=dsgdata3;end
    end
    if(SID_SPEC(n).SID'=='INER')
        INER.nchan=SID_SPEC(n).NumChan;
        INER.SensorType=SID_SPEC(n).SensorType;
        
        if(n==1) INER.data=dsgdata0; INER.SPus=SID_SPEC(1).SPus;end
        if(n==2) INER.data=dsgdata1; INER.SPus=SID_SPEC(2).SPus;end

        if(n==3) INER.data=dsgdata2; INER.SPus=SID_SPEC(3).SPus;end

        if(n==4) INER.data=dsgdata3; INER.SPus=SID_SPEC(4).SPus;end  
        
        % Check SID_SPEC for sensors saved
        k=0;
        if(bitand(INER.SensorType,32))
            for k=1:3  %accel
             INER.accel=[INER.accel accel_cal.*(INER.data(k:INER.nchan:end))];  %calibrate accelerometer
            end
        end
        m=k;
         if(bitand(INER.SensorType,16))
            for m=k+1:k+3  %magnetometer
             INER.mag=[INER.mag mag_cal.*(INER.data(m:INER.nchan:end))];  %calibrate magnetometer
            end
         end
         if(bitand(INER.SensorType,8)) 
             for j=m+1:m+3  %gyros
             INER.gyro=[INER.gyro gyro_cal.*(INER.data(j:INER.nchan:end))];  %calibrate gyro
             end
         end
    end
    if(SID_SPEC(n).SID'=='PTMP')
        PTMP.nchan=SID_SPEC(n).NumChan;
        if(n==1) PTMP.data=dsgdata0;PTMP.SPus=SID_SPEC(1).SPus;end
        if(n==2) PTMP.data=dsgdata1;PTMP.SPus=SID_SPEC(2).SPus;end
        if(n==3) PTMP.data=dsgdata2;PTMP.SPus=SID_SPEC(3).SPus;end
        if(n==4) INER.data=dsgdata3;PTMP.SPus=SID_SPEC(4).SPus;end
    end
end

if(isempty(PTMP)==0)
    % read Pressure and Temperature calibration coefficients


    FileName='PRESSTMP.CAL';
 
    fid=fopen(FileName);
    
    if(fid<1)
        disp('Press/Temp Calibration file not found');
         [FileName,PathName,FilterIndex] = uigetfile({'*.cal','cal files (*.cal)'},'Select presstemp.cal Calibration File');
          if isequal(FileName,0)|isequal(PathName,0)
            return
          end   
    end
    PSENS=fread(fid,1,'uint16');
    POFF=fread(fid,1,'uint16');
    TCSENS=fread(fid,1,'uint16');
    TCOFF=fread(fid,1,'uint16');
    TREF=fread(fid,1,'uint16');
    TEMPSENS=fread(fid,1,'uint16');
    fclose(fid);

    counter=1;
    temperature=[];
    pressure=[];
    pressraw=[];
    tempraw=[];
    for n=1:6:length(PTMP.data)
       p2bits=bitor(bitshift(uint64(PTMP.data(n)),16), bitshift(uint64(PTMP.data(n+1)),8));
       D1=double(bitor(p2bits,uint64(PTMP.data(n+2))));  % Pressure measurement

       t2bits=bitor(bitshift(uint64(PTMP.data(n+3)),16), bitshift(uint64(PTMP.data(n+4)),8));
       D2=double(bitor(t2bits,uint64(PTMP.data(n+5))));  % Temperature measurement 

      pressraw(counter)=D1; 
      tempraw(counter)=D2; 

      dT=D2-TREF*256;
      temperature(counter)= (2000+dT*TEMPSENS/8388608)/100; 

      OFF=POFF*65536+(TCOFF*dT)/128; 
      SENS=PSENS*32768+(dT*TCSENS)/256;

      pressure(counter)=(D1*SENS/2097152-OFF)/81920; % mbar (i.e. a value of 1 = 1 mbar = ~1 cm depth resolution) 

       counter=counter+1;
    end
    
    depth=(surfacepress-pressure)/100;  %estimate of depth in m
end


% Sid_SPEC.CircType Binary flag structure for sensors (sensor; bit set)
% Analog ADC6 = 0
% temperature = 1
% Pressure = 2
% gyro = 3
% magnetometer = 4
% accelerometer internal = 5
% acceleromter external = 6
% external flex = 7


% Order Data are written to file
% "ADC6" A0, Accel Ext
% "PTMP" Pressure, Temp
% "INER" Accel Int, Comp, Gyro

ot_plot;

