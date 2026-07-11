function outPath = phase_artifact_file(phaseName, artifactKind, filename)
%PHASE_ARTIFACT_FILE Return a fully qualified artifact path for a phase.

paths = phase_artifacts(phaseName);
artifactKind = char(string(artifactKind));
filename = char(string(filename));

if ~isfield(paths, artifactKind)
    error('Unknown artifact kind "%s" for phase "%s".', artifactKind, phaseName);
end

outPath = fullfile(paths.(artifactKind), filename);
end