function outPath = write_phase_table(phaseName, filename, T)
%WRITE_PHASE_TABLE Write a table into a phase CSV artifact.

outPath = phase_artifact_file(phaseName, 'csv', filename);
writetable(T, outPath);
end