function [strO] = str_comma_sep(strI, sep)
% str_comma_sep inputs the separator str between groups of three digits to the left of the decimal point. 
% 
% strI is the input foating point decimal string.  Other formats are not
% supported such as Scientific Notation, Engineering, and Base 10 
% Exponential.  
% 
% sep is the new separator string, typically a comma.
% 
% strO is the output string with the separators string added between groups
% of three digits to the left of the decimal point.  
% 
% 
% % Example
% [strO] = str_comma_sep(pi*10^6, ',');
% 

if nargin  < 2 || isempty(sep) || ~ischar(sep) 
    sep=',';
end


sep_only_to_left=1;

if sep_only_to_left == 1
    
    C = regexpi(strI,'^(\D*\d{0,3})(\d{3})*(\D\d*)?$','once','tokens');
    if numel(C)
        C = [C{1},regexprep(C{2},'(\d{3})',',$1'),C{3}];
    end
    
    strO=C;
    
else
    
    % sep string on both left and right of decimal point for all groups of
    % three digits
    
    % https://www.mathworks.com/matlabcentral/fileexchange/52832-num2sepstr
    strO = regexprep(strI,'[0-9](?=(?:[0-9]{3})+(?![0-9]))',['$&',sep]);
    
end




