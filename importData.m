function record = importData(filename)

if nargin==0
    error('filename missing');
end

record.filename = filename;

d=record.filename;
d=strrep(d,'simulatorlog',''); %  
d=strrep(d,'.txt',''); %


rowcount = readNumberOfLines(filename);

[variables,varnames,lineok] = readRawData(filename,rowcount);

IIlineok = find(lineok>0);

for i=1:size(varnames,1)
    varname = varnames{i};
    
    rr = textscan(varname,'%s',-1,'delimiter','.'); rr=rr{1};
    tt = textscan(rr{size(rr,1)}, '%s', -1, 'delimiter', '[]'); %  
    tt = tt{1};
    if size(tt,1) == 2
        d = tt{2};
        d = eval(d);
        d = d+1;
    else
        d = 1;
    end
    
    if size(rr,1) == 3
        record.(genvarname(rr{1})).(genvarname(rr{2})).(genvarname(tt{1}))(:,d) = variables(IIlineok,i);
    elseif size(rr,1) == 2
        record.(genvarname(rr{1})).(genvarname(tt{1}))(:,d) = variables(IIlineok,i);
    else
        record.(genvarname(varname))(:,d) = variables(IIlineok,i);
    end
end


end

%%
function N = readNumberOfLines(filename)

[fid, message] = fopen(filename);

if(fid < 0)
    disp(message);
    N = -1;
    return;
end

rowcount = 0;
tline = fgetl(fid); 
tline = fgetl(fid); 
while ischar(tline)
    rowcount = rowcount + 1;
    
    tline = fgetl(fid);
end

N = rowcount;

end

%% 

function [variables,varnames,lineok] = readRawData(filename,rowcount)

[fid, message] = fopen(filename);

if(fid < 0)
    disp(message);
    return;
end

headerline = fgetl(fid);

varnames = textscan(headerline,'%s',-1,'delimiter',';')';
varnames = varnames{1};

variables = zeros(rowcount,size(varnames,1));
lineok = zeros(rowcount,1);

for i = 1:rowcount
    line = fgetl(fid);
    
    line = strrep(line,'Infinity','Inf');
    line = strrep(line,'False','0');
    line = strrep(line,'True','1');
        
    if ~isempty(line)
        varcel = textscan(line,'%f',-1,'delimiter',';');
        varcel = varcel{1};
        variables(i,:) = varcel(:)';
        lineok(i) = 1;
    end
end

fclose(fid);

end