function [DF_HEAD, SID_SPEC, SID_REC,pos]=oDSG(FileName)
% Program to load DSG files from OpenTag and plot them

fid=fopen(FileName);
if(fid<1)
    disp('Unable to Open File');
    FileName
    return
end 

% Read in DF_Head
DF_HEAD.Version = fread(fid,1,'uint32');
DF_HEAD.UserID = fread(fid,1,'uint32');
DF_HEAD.sec = fread(fid,1,'uint8');
DF_HEAD.min = fread(fid,1,'uint8');
DF_HEAD.hour = fread(fid,1,'uint8');
DF_HEAD.day = fread(fid,1,'uint8');
DF_HEAD.mday = fread(fid,1,'uint8');
DF_HEAD.month = fread(fid,1,'uint8');
DF_HEAD.year = fread(fid,1,'uint8'); 
DF_HEAD.timezone = fread(fid,1,'int8');

%DF_HEAD.NU = fread(fid,16,'uint32');

if(DF_HEAD.Version>=1010) 
    DF_HEAD.Lat=fread(fid,1,'float32');
    DF_HEAD.Lon=fread(fid,1,'float32');
    DF_HEAD.depth=fread(fid,1,'float32');
    DF_HEAD.DSGcal=fread(fid,1,'float32');
    DF_HEAD.hydroCal=fread(fid,1,'float32');
    DF_HEAD.lpFilt=fread(fid,1,'float32');
end
    
pos = ftell(fid);
  
% Read in SID_SPECS until get all zeroes
notdone=1;
SID_SPEC=[];
nSIDSPEC=1;
while(notdone)
    SID_SPEC(nSIDSPEC).SID = fread(fid,4,'uint8=>char');
    SID_SPEC(nSIDSPEC).nBytes = fread(fid,1,'uint32');
    SID_SPEC(nSIDSPEC).NumChan = fread(fid,1,'uint32');
    SID_SPEC(nSIDSPEC).StoreType = fread(fid,1,'uint32');
    SID_SPEC(nSIDSPEC).SensorType = fread(fid,1,'uint32');
    SID_SPEC(nSIDSPEC).DForm = fread(fid,1,'uint32');
    SID_SPEC(nSIDSPEC).SPus = fread(fid,1,'uint32'); % Sample period (us) x 256
    SID_SPEC(nSIDSPEC).RECPTS=fread(fid,1,'uint32');
    SID_SPEC(nSIDSPEC).RECINT=fread(fid,1,'uint32');
    
    if(SID_SPEC(nSIDSPEC).nBytes==0)
        notdone=0;
    end
    nSIDSPEC=nSIDSPEC+1; 
end
nSIDSPEC=nSIDSPEC-1;
SID_SPEC(nSIDSPEC)=[];  % delete last one with all zeroes
nSIDSPEC=nSIDSPEC-1;

% Read in next SID_REC header and data
eofstat=0;
reccounter=0;
SID_REC=[]; 

while(eofstat==0)
    reccounter=reccounter+1;
    SID_REC(reccounter).nSID=fread(fid,1,'uint8');
    SID_REC(reccounter).Chan=fread(fid,1,'uint8');
    SID_REC(reccounter).nbytes=fread(fid,1,'uint32');  %  Number of bytes recorded since start of sampling for this SID
    if (DF_HEAD.Version>9999)
        SID_REC(reccounter).nbytes_2=fread(fid,1,'uint32');  %roll counter
    end
    cur_sid=(SID_REC(reccounter).nSID)+1;
    if(cur_sid >0 & cur_sid<4)         
        if(SID_SPEC(cur_sid).DForm==2)
            nsamples=(SID_SPEC(cur_sid).nBytes)/2;  %/2 because in bytes
            SID_REC(reccounter).data=fread(fid,nsamples,'int16');
        end
        if(SID_SPEC(cur_sid).DForm==3)
            nsamples=SID_SPEC(cur_sid).nBytes;
            SID_REC(reccounter).data=fread(fid,nsamples,'uint8');  % 24-bit samples read in 8 bits at a time
        end
    else
        SID_REC(reccounter)=[]; %last one was bad so delete this entry
        reccounter=reccounter-1;
    end
     
    ftell(fid);
    eofstat = feof(fid);
end

fclose(fid);
