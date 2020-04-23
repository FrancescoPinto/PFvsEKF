function save_matrix_to_file(file_name,matrix)

fid = fopen(file_name,'w');
fwrite(fid,matrix,'double');
fclose(fid);
%{
for i = 1:size(matrix,2)
    fprintf(fid,'%20.18f \t',matrix(:,i));
    fprintf(fid,'\n');
end
fclose(fid);
%}
end

