function meta = voxelize(source_type, input_path, varargin)
%VOXELIZE  统一前端: 把 STL 或拓扑优化 mat 文件转成 Abaqus 输入网格.
%
%   meta = VOXELIZE('stl',  stl_path,  ...) 走 STL 体素化路径
%   meta = VOXELIZE('topo', mat_path,  ...) 走 SIMP 拓扑优化路径
%
%   两种模式都输出 voxel_grid.inp + voxel_grid.npz, 可直接在 Abaqus
%   导入做应力分析, 然后用 abaqus_odb_to_mat.py 提取应力到 .mat,
%   接入 voxel_refinement_from_test.m 等下游脚本.
%
%   公共可选参数 (Name-Value):
%     'OutputInp'    .inp 输出路径    (默认 'voxel_grid.inp')
%     'OutputNpz'    .npz 输出路径    (默认 'voxel_grid.npz')
%     'ElementType'  'C3D8R' / 'C3D8' / 'C3D8I'   (默认 'C3D8R')
%     'Youngs'       杨氏模量        (默认 70000)
%     'Poisson'      泊松比          (默认 0.3)
%     'Density'      密度            (默认 2.7e-9)
%     'PythonExe'    Python 解释器路径
%
%   STL 模式专用:
%     'VoxelSize'    体素边长 (必填)
%     'RotateAxis'   'y-to-z' 等坐标轴预设 (默认 'none')
%     'Padding'      外侧空体素层数 (默认 1)
%     'FixMesh'      是否修复 mesh   (默认 true)
%
%   Topo 模式专用:
%     'VoxelSize'           体素边长 (默认 1.0)
%     'DensityThreshold'    阈值, xPhys > 此值为实心 (默认 0.5)
%
%   示例:
%       % STL 模式 (体素 1mm, 默认 Z-up)
%       voxelize('stl', 'part.stl', 'VoxelSize', 1.0);
%
%       % STL 模式 (Y-up STL 需要旋转)
%       voxelize('stl', 'part.stl', 'VoxelSize', 1.0, 'RotateAxis', 'y-to-z');
%
%       % 拓扑优化 mat 直接转
%       voxelize('topo', 'simp_result.mat', 'DensityThreshold', 0.5);
%
%   完整工作流:
%       voxelize('topo', 'simp_result.mat');                  % 1) 生成 .inp/.npz
%       % 2) 在 Abaqus/CAE 里 Import .inp, 加 BC/Load/Step/Job
%       % 3) 命令行: abaqus python abaqus_odb_to_mat.py --odb job.odb --npz voxel_grid.npz
%       run('voxel_refinement_from_test.m');                  % 4) 接入管线
%       run('generate_reference_surface.m');
%       run('slice_refined_model_v6.m');
%       run('all_layers_path_generation_v6.m');

% ---- parse ----
p = inputParser;
p.KeepUnmatched = false;
p.addRequired('source_type', @(s) any(strcmpi(s, {'stl','topo'})));
p.addRequired('input_path', @(s) ischar(s) || isstring(s));
% Common
p.addParameter('OutputInp', 'voxel_grid.inp', @(s) ischar(s) || isstring(s));
p.addParameter('OutputNpz', 'voxel_grid.npz', @(s) ischar(s) || isstring(s));
p.addParameter('ElementType', 'C3D8R', @(s) ismember(char(s), {'C3D8','C3D8R','C3D8I'}));
p.addParameter('Youngs', 70000.0, @isscalar);
p.addParameter('Poisson', 0.3, @isscalar);
p.addParameter('Density', 2.7e-9, @isscalar);
p.addParameter('PythonExe', '', @(s) ischar(s) || isstring(s));
% STL-only (also reused by topo)
p.addParameter('VoxelSize', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
p.addParameter('RotateAxis', 'none', @(s) ischar(s) || isstring(s));
p.addParameter('Padding', 1, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addParameter('FixMesh', true, @islogical);
% Topo-only
p.addParameter('DensityThreshold', 0.5, @isscalar);
p.parse(source_type, input_path, varargin{:});
opt = p.Results;

src        = lower(char(opt.source_type));
input_path = char(opt.input_path);
out_inp    = char(opt.OutputInp);
out_npz    = char(opt.OutputNpz);
py_exe     = char(opt.PythonExe);

% ---- voxel size required for stl ----
if strcmp(src, 'stl') && isempty(opt.VoxelSize)
    error('voxelize:NeedVoxelSize', ...
        "STL mode requires 'VoxelSize' parameter.");
end
voxel_size = opt.VoxelSize;
if isempty(voxel_size), voxel_size = 1.0; end   % topo default

% ---- locate voxelize.py ----
this_dir = fileparts(mfilename('fullpath'));
py_script = fullfile(this_dir, 'voxelize.py');
if ~exist(py_script, 'file')
    if exist(fullfile(pwd, 'voxelize.py'), 'file')
        py_script = fullfile(pwd, 'voxelize.py');
    else
        error('voxelize:NotFound', 'voxelize.py not found near %s', this_dir);
    end
end

% ---- locate python ----
if isempty(py_exe)
    if ispc
        candidates = {'python', 'python3'};
    else
        candidates = {'python3', 'python'};
    end
    py_exe = '';
    for ii = 1:numel(candidates)
        [st, ~] = system([candidates{ii} ' --version']);
        if st == 0
            py_exe = candidates{ii};
            break;
        end
    end
    if isempty(py_exe)
        error('voxelize:NoPython', ...
            'Python not found. Pass ''PythonExe'' or add to PATH.');
    end
end

if ~exist(input_path, 'file')
    error('voxelize:NoInput', 'Input not found: %s', input_path);
end

% ---- build command ----
common = sprintf(' -i "%s" -m "%s" -e %s -E %g -v %g -d %g', ...
    out_inp, out_npz, char(opt.ElementType), ...
    opt.Youngs, opt.Poisson, opt.Density);

if strcmp(src, 'stl')
    sub_args = sprintf(' -s %.10g -p %d', voxel_size, opt.Padding);
    rotate_axis = char(opt.RotateAxis);
    if ~isempty(rotate_axis) && ~strcmpi(rotate_axis, 'none')
        sub_args = [sub_args, sprintf(' -r %s', rotate_axis)];
    end
    if ~opt.FixMesh
        sub_args = [sub_args, ' --no-fix'];
    end
    cmd = sprintf('"%s" "%s" from-stl "%s"%s%s', ...
        py_exe, py_script, input_path, sub_args, common);
else  % topo
    sub_args = sprintf(' -s %.10g -t %.10g', voxel_size, opt.DensityThreshold);
    cmd = sprintf('"%s" "%s" from-topo "%s"%s%s', ...
        py_exe, py_script, input_path, sub_args, common);
end

fprintf('[voxelize] running:\n  %s\n\n', cmd);
[status, ~] = system(cmd, '-echo');
if status ~= 0
    error('voxelize:PyFailed', 'Python script failed (exit %d)', status);
end

if ~exist(out_inp, 'file') || ~exist(out_npz, 'file')
    error('voxelize:NoOutput', 'Output files not generated.');
end

% ---- return metadata ----
meta = struct();
meta.source_type = src;
meta.input_path  = input_path;
meta.inp_path    = out_inp;
meta.npz_path    = out_npz;
meta.voxel_size  = voxel_size;

fprintf('\n[voxelize] DONE (%s mode).\n', src);
fprintf('  .inp:  %s\n', out_inp);
fprintf('  .npz:  %s\n', out_npz);
fprintf('\nNext steps:\n');
fprintf('  1) Abaqus/CAE -> File -> Import -> Model -> %s\n', out_inp);
fprintf('  2) Apply BCs (use N_XMIN/XMAX/YMIN/YMAX/ZMIN/ZMAX node sets)\n');
fprintf('  3) Apply loads, create Step, submit Job\n');
fprintf('  4) abaqus python abaqus_odb_to_mat.py --odb job.odb --npz %s\n', out_npz);
fprintf('  5) MATLAB: run(''voxel_refinement_from_test.m'')\n');

end
