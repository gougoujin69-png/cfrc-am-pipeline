function pipeline_inventory()
% Fast inventory (header-only) of all pipeline inputs/intermediates.
files = {
  'all_layers_path_results_v3.mat',      'results (carbon input)'
  'voxel_refined_latest.mat',            'refined_data (voxel)'
  'support_paths_step0.mat',             'support_data (Step0)'
  'resin_paths_step0.mat',               'resin_data (Step0 alt)'
  'all_layers_path_carbon_resin.mat',    'all_layers_data (merged)'
  'all_layers_path_carbon_resin_raw.mat','all_layers_data (merged raw)'
  'Modified_printing_path.mat',          'all_layers_data (Step2)'
  'Manufacturing_printing_path.mat',     'all_layers_data (Step3.1 FINAL)'
  'base_support_paths.mat',              'base_support_data (Step4)'
  'pipeline_funcs/GPRmodel.mat',         'gprMdlv1 (drag comp)'
  'Base_Reduced.stl',                    'lowered base STL'
  'Base_Model.stl',                      'original base STL'
};
fprintf('%-42s %-10s %-8s  %s\n','file','exists','MB','top-level vars');
fprintf('%s\n', repmat('-',1,110));
for i=1:size(files,1)
    f = files{i,1};
    if exist(f,'file')~=2
        fprintf('%-42s %-10s\n', f, 'MISSING'); continue;
    end
    d = dir(f); mb = d.bytes/1e6;
    vars = '';
    if endsWith(f,'.mat')
        try
            w = whos('-file', f);
            nm = {w.name};
            vars = strjoin(nm(1:min(6,numel(nm))), ', ');
        catch ME
            vars = ['<whos failed: ' ME.message '>'];
        end
    end
    fprintf('%-42s %-10s %8.1f  %s\n', f, 'yes', mb, vars);
end

% layer counts for the key structs (cheap: load only the struct field sizes)
fprintf('\n--- layer counts / key fields ---\n');
probe('Manufacturing_printing_path.mat','all_layers_data');
probe('Modified_printing_path.mat','all_layers_data');
probe('all_layers_path_carbon_resin.mat','all_layers_data');
end

function probe(f, v)
if exist(f,'file')~=2, fprintf('%-40s : missing\n', f); return; end
try
    m = matfile(f);
    s = size(m.(v));
    nL = max(s);
    a = m.(v)(1, min(1,s(1))>0 + 0 + 1);  %#ok<NASGU>
    fn = fieldnames(m.(v));
    fprintf('%-40s : %d layers; fields = %s\n', f, nL, strjoin(fn(:).', ', '));
catch ME
    fprintf('%-40s : probe failed (%s)\n', f, ME.message);
end
end
