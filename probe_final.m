function probe_final()
% Properly probe the FINAL print data to size the Excel/MOD job.
S = load('Manufacturing_printing_path.mat','all_layers_data');
ald = S.all_layers_data;
nL = numel(ald);
fn = fieldnames(ald);
fprintf('Manufacturing_printing_path.mat: %d layers\n', nL);
fprintf('fields: %s\n', strjoin(fn(:).', ', '));
hasC = isfield(ald,'connected_paths');
hasR = isfield(ald,'resin_connected_paths');
hasPC = isfield(ald,'pointCloud_data');
fprintf('has connected_paths=%d  resin_connected_paths=%d  pointCloud_data=%d\n', hasC, hasR, hasPC);
ncSeg=0; npC=0; nrSeg=0; npR=0; nLcarbon=0;
for k=1:nL
    if hasC && ~isempty(ald(k).connected_paths)
        c = ald(k).connected_paths; nLcarbon = nLcarbon + 1;
        ncSeg = ncSeg + numel(c);
        for i=1:numel(c), npC = npC + size(c{i},1); end
    end
    if hasR && ~isempty(ald(k).resin_connected_paths)
        r = ald(k).resin_connected_paths;
        nrSeg = nrSeg + numel(r);
        for i=1:numel(r), npR = npR + size(r{i},1); end
    end
end
fprintf('\nCARBON: layers-with-paths=%d  segments=%d  total points=%d\n', nLcarbon, ncSeg, npC);
fprintf('RESIN : segments=%d  total points=%d\n', nrSeg, npR);
% sample layer-10 first carbon segment shape + z range
for k=1:nL
    if hasC && ~isempty(ald(k).connected_paths)
        c = ald(k).connected_paths{1};
        fprintf('sample layer %d seg1: %dx%d, X[%.1f %.1f] Y[%.1f %.1f] Z[%.1f %.1f]\n', ...
            k, size(c,1), size(c,2), min(c(:,1)),max(c(:,1)),min(c(:,2)),max(c(:,2)),min(c(:,3)),max(c(:,3)));
        break;
    end
end
end
