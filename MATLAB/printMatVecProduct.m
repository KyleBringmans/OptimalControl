function printMatVecProduct(fileID, A , alpha , startIndexX , startIndexY , nameA , nameX , nameY , nameZ)
%PRINTMATVEC prints a matrix-vector product (hard-coded)
%
% y := alpha * z + A * x;
%
%Syntax:
% printMatVec(f, A, alpha, startindexX, startIndexY, nameA, nameX, nameY)
% printMatVec(f, A, alpha, startindexX, startIndexY, nameA, nameX, nameY, nameZ)
%
%Arguments:
% f file handler (use fopen to produce one)
% A matrix A (the dimensions of x, y and z are inferred from the
% dimensions of A)
% alpha scalar
% startIndexX starting index of X
% startIndexY starting index of Y
% nameA name of variable A
% nameX name of variable x
% nameY name of variable y (cannot be equal to nameX)
% nameZ name of variable z (can be equal to nameY). If nameZ is not
% specified and alpha is not set to zero, then it is assumed that
% nameZ = nameY and the operation y := alpha * y + A * x is
% generated.
%if strcmp(nameX,nameY) == 1
%    error('nameX cannot be the same as nameY')
%end
if nargin <7
    error('not enough input arguments')
end
if nargin == 8
    nameZ = nameY;
end

[nbRows, nbCols] = size(A);
format = strings(nbRows,1);
% if alpha == 0
%     if nbCols == 1
%         if nbRows == 1
%             formatmatrix = cell(nbRows,1+nbCols*2);
%         else
%             formatmatrix = cell(nbRows,2+nbCols*2);
%         end
%     else
%         if nbRows == 1
%             formatmatrix = cell(nbRows,1+nbCols*3);
%         else
%             formatmatrix = cell(nbRows,2+nbCols*3);
%         end
%     end
% elseif any(any(A)) == 0
%     formatmatrix = cell(nbRows,5);
% else
%     formatmatrix = cell(nbRows,5+nbCols*3);
% end

% Documentation and printing of file
if iscell(nameX) == 0
if any(any(A)) == 0
    documentationFormatMatrix = cell(1,3);
    documentationFormatMatrix{1,1}= nameY;
    documentationFormatMatrix{1,2}= alpha;
    documentationFormatMatrix{1,3}= nameZ;
    fprintf(fileID,'/* %s <-- %16g * %s */\r\n',documentationFormatMatrix{1,1:end});

elseif alpha == 0
    documentationFormatMatrix = cell(1,3);
    documentationFormatMatrix{1,1}= nameY;
    documentationFormatMatrix{1,2}= nameA;
    documentationFormatMatrix{1,3}= nameX;
    fprintf(fileID,'/* %s <-- %s * %s */\r\n',documentationFormatMatrix{1,1:end});
else
    documentationFormatMatrix = cell(1,5);
    documentationFormatMatrix{1,1}= nameY;
    documentationFormatMatrix{1,2}= alpha;
    documentationFormatMatrix{1,3}= nameZ;
    documentationFormatMatrix{1,4}= nameA;
    documentationFormatMatrix{1,5}= nameX;
    fprintf(fileID,'/* %s <-- %16g * %s + %s * %s */\r\n',documentationFormatMatrix{1,1:end});
    %   fprintf(fileID,'/* %s <-- %g * %s + %s * %s */\r\n',documentationFormatMatrix{1,1:end});

end


formatmatrixrows = cell(nbRows,1);
for i= 1:nbRows
    formatmatrix = {};
    formatmatrix{end+1} = nameY;
    if nbRows ~= 1
        formatmatrix{end+1} = i-1+startIndexY;
    end
    if (alpha == 0 & A == 0)
        if nbRows == 1
            format(i,1) = ('%s = 0');
        else
            format(i,1) = ('%s[%u] = 0');
        end
    elseif alpha == 0
        if nbRows == 1
            format(i,1) = ('%s =');
        else
            format(i,1) = ('%s[%u] =');
        end
    else
        if nbRows == 1 & nbCols == 1
            formatmatrix{end+1} = alpha;
            formatmatrix{end+1} = nameZ;
            format(i,1) = ('%s = (%16g) * %s');
        elseif nbRows == 1
            formatmatrix{end+1} = alpha;
            formatmatrix{end+1} = nameZ;
            formatmatrix{end+1} = i-1+startIndexY;
            format(i,1) = ('%s = (%16g) * %s[%u]');
        elseif nbCols == 1
            formatmatrix{end+1} = alpha;
            formatmatrix{end+1} = nameZ;
            format(i,1) = ('%s[%u] = (%16g) * %s');
        else
            formatmatrix{end+1} = alpha;
            formatmatrix{end+1} = nameZ;
            formatmatrix{end+1} = i-1+startIndexY;
            format(i,1) = ('%s[%u] = (%16g) * %s[%u]');
        end
    end
    if any(any(A)) == 1
        for j = 1:nbCols
            if nbCols == 1
                formatmatrix{end+1} = A(i,j);
                formatmatrix{end+1} = nameX;
                format(i,1) = strcat(format(i,1), '+ (%16g) * %s');
            else
                formatmatrix{end+1} = A(i,j);
                formatmatrix{end+1} = nameX;
                formatmatrix{end+1} = j-1+startIndexX;
                format(i,1) = strcat(format(i,1), ' + (%16g) * (%s[%u])');
            end
        end
    end
    format(i,1) = strcat(format(i,1), ';\r\n');
    fprintf(fileID,format(i,1),formatmatrix{1:end});
end
end

%function for cellarray
if iscell(nameX) == 1
if any(any(A)) == 0
    documentationFormatMatrix = cell(1,3);
    documentationFormatMatrix{1,1}= nameY;
    documentationFormatMatrix{1,2}= alpha;
    documentationFormatMatrix{1,3}= nameZ;
    %fprintf(fileID,'/* %s <-- %16g * %s */\r\n',documentationFormatMatrix{1,1:end});

elseif alpha == 0
    documentationFormatMatrix = cell(1,3);
    documentationFormatMatrix{1,1}= nameY;
    documentationFormatMatrix{1,2}= nameA;
    documentationFormatMatrix{1,3}= nameX;
   % fprintf(fileID,'/* %s <-- %s * %s */\r\n',documentationFormatMatrix{1,1:end});
else
    documentationFormatMatrix = cell(1,5);
    documentationFormatMatrix{1,1}= nameY;
    documentationFormatMatrix{1,2}= alpha;
    documentationFormatMatrix{1,3}= nameZ;
    documentationFormatMatrix{1,4}= nameA;
    documentationFormatMatrix{1,5}= nameX;
    %fprintf(fileID,'/* %s <-- %16g * %s + %s * %s */\r\n',documentationFormatMatrix{1,1:end});
    %   fprintf(fileID,'/* %s <-- %g * %s + %s * %s */\r\n',documentationFormatMatrix{1,1:end});

end


formatmatrixrows = cell(nbRows,1);
for i= 1:nbRows
    formatmatrix = {};
    formatmatrix{end+1} = nameY;
    if nbRows ~= 1
        formatmatrix{end+1} = i-1+startIndexY;
    end
    if (alpha == 0 & A == 0)
        if nbRows == 1
            format(i,1) = ('%s = 0');
        else
            format(i,1) = ('%s[%u] = 0');
        end
    elseif alpha == 0
        if nbRows == 1
            format(i,1) = ('%s =');
        else
            format(i,1) = ('%s[%u] =');
        end
    else
        if nbRows == 1 & nbCols == 1
            formatmatrix{end+1} = alpha;
            formatmatrix{end+1} = nameZ;
            format(i,1) = ('%s = (%16g) * %s');
        elseif nbRows == 1
            formatmatrix{end+1} = alpha;
            formatmatrix{end+1} = nameZ;
            formatmatrix{end+1} = i-1+startIndexY;
            format(i,1) = ('%s = (%16g) * %s[%u]');
        elseif nbCols == 1
            formatmatrix{end+1} = alpha;
            formatmatrix{end+1} = nameZ;
            format(i,1) = ('%s[%u] = (%16g) * %s');
        else
            formatmatrix{end+1} = alpha;
            formatmatrix{end+1} = nameZ;
            formatmatrix{end+1} = i-1+startIndexY;
            format(i,1) = ('%s[%u] = (%16g) * %s[%u]');
        end
    end
    if any(any(A)) == 1
        for j = 1:nbCols
            if A(i,j) ~=0
                if nbCols == 1
                    formatmatrix{end+1} = A(i,j);
                    formatmatrix{end+1} = nameX{j};
                    format(i,1) = strcat(format(i,1), '+ (%16g) * %s');
                else
                    formatmatrix{end+1} = A(i,j);
                    formatmatrix{end+1} = nameX{j};
                    %formatmatrix{end+1} = j-1+startIndexX;
                    format(i,1) = strcat(format(i,1), ' + (%16g) * (%s)');
                end
            end
        end
    end
    format(i,1) = strcat(format(i,1), ';\r\n');
    fprintf(fileID,format(i,1),formatmatrix{1:end});
end
end


% 
% for i = 1:nbRows
%     fprintf(fileID,format(i,1),formatmatrix{i,1:end});
% end

    

        