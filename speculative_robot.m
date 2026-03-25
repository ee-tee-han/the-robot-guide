clear;clc;% Task 3 Subtask 2 - Speculative Programming with Gemini AI
% Simulates two people debating landmarks near the robot. The robot listens,
% pre-computes paths mid-conversation, and answers instantly when asked.
% Get a free API key at: https://aistudio.google.com/app/apikey

clc;

API_KEY    = 'AIzaSyCVDGsn9Rz3FAxtEqRm3pJJFHabL1Mr8xY';
MODEL      = 'gemini-2.5-flash-lite';
TIME_SCALE = 0.3;

if ~exist('adj', 'var') || ~exist('node_x', 'var')
    fprintf('loading map...\n\n');
    run('map_search_algorithms.m');
end

ll_keys = SinglyLinkedList();
for i = 1:numKey
    ll_keys.addLast(struct('name', key_label{i}, 'x', key_x(i), 'y', key_y(i), 'type', 'key'));
end
kd_keys = KDTreeSearcher([key_x, key_y]);

all_landmarks = {'One Pool Street', 'Marshgate', 'ArcelorMittal Orbit', ...
                 'West Ham Stadium', 'Copper Box Arena', 'Here East', 'Aquatics Centre'};
% trim to however many key points this dataset actually produced
landmarks = all_landmarks(1:min(numKey, length(all_landmarks)));
if numKey > length(all_landmarks)
    for i = length(all_landmarks)+1 : numKey
        landmarks{i} = sprintf('K%d', i);
    end
end

landmark_list = '';
for i = 1:numKey
    landmark_list = [landmark_list, sprintf('K%d: %s\n', i, landmarks{i})];
end

fallback = { ...
    {0.0, 'ALEX', 'OK so where should we head first - Aquatics Centre or West Ham Stadium?'}; ...
    {1.6, 'SAM',  'Aquatics Centre for sure! Then the ArcelorMittal Orbit after?'}; ...
    {1.2, 'ALEX', 'Love the Orbit! Should we add Copper Box Arena too?'}; ...
    {1.5, 'SAM',  'Hmm - actually I think we should start at West Ham Stadium.'}; ...
    {1.4, 'ALEX', 'So West Ham Stadium first, then Aquatics Centre, then the Orbit?'}; ...
    {1.6, 'SAM',  'Yes but Marshgate is right next to West Ham Stadium - easy detour.'}; ...
    {1.3, 'ALEX', 'Good shout. Marshgate then West Ham Stadium then the Orbit.'}; ...
    {1.7, 'SAM',  'What about Here East and One Pool Street while we are at it?'}; ...
    {1.2, 'ALEX', 'Here East feels out of the way... One Pool Street is fine though.'}; ...
    {1.6, 'SAM',  'Agreed - Marshgate, West Ham Stadium, Orbit, Aquatics, One Pool Street.'}; ...
    {1.3, 'ALEX', 'Perfect. Robot, please confirm the best route for all of that!'}; ...
};

% longest phrases first to avoid partial matches
kw_map = { ...
    'arcelormittal orbit', 3; 'west ham stadium', 4; 'copper box arena', 5; ...
    'one pool street',     1; 'aquatics centre',  7; 'here east',        6; ...
    'marshgate',           2; 'arcelormittal',    3; 'copper box',       5; ...
    'west ham',            4; 'aquatics',         7; 'orbit',            3; ...
    'stadium',             4; 'pool street',      1; ...
};

fprintf('\n--- Speculative Robot (GenAI) ---\n');
use_ai = ~isempty(API_KEY);

if use_ai
    fprintf('Using Gemini: %s  (time scale: %.1f)\n\n', MODEL, TIME_SCALE);
else
    fprintf('No API key - using fallback conversation\n\n');
end

conv = {};
MIN_GAP_S = 13.0;
t_rl = tic;
last_gemini_rl = -MIN_GAP_S;

if use_ai
    fprintf('Asking Gemini to write the conversation...\n');

    sys_prompt = 'You generate realistic conversations for a robot navigation demo. Always respond with valid JSON only, no markdown.';
    usr_prompt = sprintf(['Generate a natural conversation between two people called ALEX and SAM ' ...
        'who are debating which landmarks to visit in Queen Elizabeth Olympic Park. ' ...
        'They should be indecisive and change their minds a few times before agreeing on an order. ' ...
        'Make it 10-12 turns and cover most of these landmarks:\n%s\n' ...
        'Return JSON like this: {"conversation":[{"speaker":"ALEX","pause_before":0.0,"text":"..."},...]}' ...
        '\npause_before is the gap in seconds before each line (realistic talking pace).'], landmark_list);

    t0 = tic;
    raw = gemini_call(API_KEY, MODEL, sys_prompt, usr_prompt);
    t_gen = toc(t0) * 1000;
    last_gemini_rl = toc(t_rl);

    if isempty(raw)
        fprintf('Gemini failed - using fallback\n\n');
        conv = fallback;
        use_ai = false;
    else
        try
            parsed = jsondecode(raw);
            turns = parsed.conversation;
            for i = 1:length(turns)
                t = turns(i);
                conv{end+1} = {t.pause_before, t.speaker, t.text};
            end
            fprintf('Got %d turns from Gemini (%.0f ms)\n\n', length(conv), t_gen);
        catch e
            fprintf('Could not parse response (%s) - using fallback\n\n', e.message);
            conv = fallback;
            use_ai = false;
        end
    end
else
    conv = fallback;
end

% warm-up: run both search structures once so MATLAB JIT compiles them
% before any timed calls, giving fair measurements throughout
cur = ll_keys.head;
while ~isempty(cur); cur = cur.next; end
knnsearch(kd_keys, [key_x(1), key_y(1)]);

spec_cache  = cell(numKey, 1);
mention_log = {};
all_mentioned = [];
waiting_area  = 1;
t_ai_total = 0;
t_kw_total = 0;
extract_log = {};

fprintf('--- Live conversation ---\n\n');
t_start = tic;

for ci = 1:length(conv)
    line     = conv{ci};
    gap      = line{1};
    speaker  = line{2};
    sentence = line{3};

    pause(gap * TIME_SCALE);

    t_now = toc(t_start);
    fprintf('[%.1fs] %s: ', t_now, speaker);
    words = strsplit(sentence, ' ');
    for w = 1:length(words)
        fprintf('%s ', words{w});
        pause(0.16 * TIME_SCALE);
    end
    fprintf('\n');

    % keyword extraction - O(K*W)
    detected_kw = [];
    txt = lower(sentence);
    used_chars = false(1, length(txt));
    t0 = tic;
    for ki = 1:size(kw_map, 1)
        phrase = kw_map{ki, 1};
        hits   = strfind(txt, phrase);
        for h = hits
            idx_range = h : h + length(phrase) - 1;
            if ~any(used_chars(idx_range))
                used_chars(idx_range) = true;
                kpt = kw_map{ki, 2};
                if kpt <= numKey && ~ismember(kpt, detected_kw)
                    detected_kw(end+1) = kpt;
                end
            end
        end
    end
    t_kw_line  = toc(t0) * 1000;
    t_kw_total = t_kw_total + t_kw_line;

    % gemini extraction - throttled to stay under rate limit
    detected_ai = [];
    t_ai_line   = 0;
    gemini_ok   = false;

    if use_ai
        since_last = toc(t_rl) - last_gemini_rl;
        if since_last < MIN_GAP_S
            pause(MIN_GAP_S - since_last);
        end

        sys2 = 'You extract landmark mentions from conversation. Always respond with valid JSON only, no markdown.';
        usr2 = sprintf(['Available landmarks:\n%s\n' ...
            'Which of these are mentioned or implied in this sentence?\n"%s"\n\n' ...
            'Reply with JSON: {"mentioned":[{"index":K,"name":"..."}]}\n' ...
            'Use an empty array if nothing matches.'], landmark_list, sentence);

        t0 = tic;
        raw2       = gemini_call(API_KEY, MODEL, sys2, usr2);
        t_ai_line  = toc(t0) * 1000;
        last_gemini_rl = toc(t_rl);

        if ~isempty(raw2)
            try
                ex = jsondecode(raw2);
                if ~isempty(ex.mentioned)
                    for m = 1:length(ex.mentioned)
                        k = ex.mentioned(m).index;
                        if k >= 1 && k <= numKey && ~ismember(k, detected_ai)
                            detected_ai(end+1) = k;
                        end
                    end
                end
                gemini_ok  = true;
                t_ai_total = t_ai_total + t_ai_line;
            catch
            end
        end
    end

    if use_ai && gemini_ok
        detected = detected_ai;
        ai_names = strjoin(arrayfun(@(k) landmarks{k}, detected_ai, 'UniformOutput', false), ', ');
        kw_names = strjoin(arrayfun(@(k) landmarks{k}, detected_kw, 'UniformOutput', false), ', ');
        if isempty(ai_names); ai_names = 'none'; end
        if isempty(kw_names); kw_names = 'none'; end
        extract_log{end+1} = struct('sentence', sentence, 'ai', ai_names, 'kw', kw_names, ...
            't_ai', t_ai_line, 't_kw', t_kw_line, ...
            'match', isequal(sort(detected_ai), sort(detected_kw)), 'failed', false);
    elseif use_ai && ~gemini_ok
        detected = detected_kw;
        extract_log{end+1} = struct('sentence', sentence, 'ai', 'FAILED', 'kw', ...
            strjoin(arrayfun(@(k) landmarks{k}, detected_kw, 'UniformOutput', false), ', '), ...
            't_ai', t_ai_line, 't_kw', t_kw_line, 'match', false, 'failed', true);
    else
        detected = detected_kw;
    end

    for pi = 1:length(detected)
        k = detected(pi);

        if isempty(spec_cache{k})
            t_detect = toc(t_start);

            % linked list nearest neighbour - O(K)
            t0 = tic;
            best_d = inf; nn_ll = '';
            cur = ll_keys.head;
            while ~isempty(cur)
                p = cur.data;
                d = sqrt((p.x - key_x(k))^2 + (p.y - key_y(k))^2);
                if d < best_d; best_d = d; nn_ll = p.name; end
                cur = cur.next;
            end
            t_ll = toc(t0) * 1000;

            % kd-tree nearest neighbour - O(log K)
            t0 = tic;
            [kd_idx, ~] = knnsearch(kd_keys, [key_x(k), key_y(k)]);
            nn_kd = key_label{kd_idx};
            t_kd  = toc(t0) * 1000;

            % dijkstra - O((V+E) log V)
            src_node = numKey + WAITING_IDX(waiting_area);
            t0 = tic;
            [sp_path, sp_dist] = dijkstra_sp(adj, src_node, k, length(adj));
            t_dijk = toc(t0) * 1000;

            spec_cache{k} = struct('path', {sp_path}, 'dist', sp_dist, ...
                't_ll', t_ll, 't_kd', t_kd, 't_dijk', t_dijk, ...
                'nn_ll', nn_ll, 'nn_kd', nn_kd, 'detected_at', t_detect);

            fprintf('   [robot] heard "%s" - computing path in background\n', landmarks{k});
            fprintf('           LL: %.4f ms  KD: %.4f ms  Dijkstra: %.4f ms  dist=%.1fm\n', t_ll, t_kd, t_dijk, sp_dist);

            mention_log{end+1} = sprintf('t=%.1fs  K%d  %-22s  LL=%.4f ms  KD=%.4f ms  Dijkstra=%.4f ms', ...
                t_detect, k, landmarks{k}, t_ll, t_kd, t_dijk);
        else
            fprintf('   [robot] "%s" already cached\n', landmarks{k});
        end

        if ~ismember(k, all_mentioned)
            all_mentioned(end+1) = k;
        end
    end
end

t_end = toc(t_start);
fprintf('\n--- conversation ended (%.1f s) ---\n', t_end);

fprintf('\nRobot answer (from cache):\n');
t0 = tic;
route   = {node_label{numKey + WAITING_IDX(waiting_area)}};
total_d = 0;
for i = 1:length(all_mentioned)
    k = all_mentioned(i);
    if ~isempty(spec_cache{k})
        route{end+1} = key_label{k};
        total_d = total_d + spec_cache{k}.dist;
    end
end
t_cached = toc(t0) * 1000;
fprintf('  Route:    %s\n', strjoin(route, ' -> '));
fprintf('  Distance: ~%.1f m\n', total_d);
fprintf('  Response time (cache): %.4f ms\n', t_cached);

fprintf('\nIf robot had waited until now to compute (non-speculative):\n');
t0 = tic;
for i = 1:length(all_mentioned)
    k        = all_mentioned(i);
    src_node = numKey + WAITING_IDX(waiting_area);
    knnsearch(kd_keys, [key_x(k), key_y(k)]);
    dijkstra_sp(adj, src_node, k, length(adj));
end
t_nonspec = toc(t0) * 1000;
fprintf('  Response time (no speculation): %.4f ms\n', t_nonspec);
fprintf('  Speedup from speculation: %.1fx\n', t_nonspec / max(t_cached, 1e-9));

fprintf('\nPre-computation timeline:\n');
for i = 1:length(mention_log)
    fprintf('  %d. %s\n', i, mention_log{i});
end
fprintf('All %d paths were ready before the user finished asking.\n', length(all_mentioned));

fprintf('\nLinked List vs KD-Tree (per detected landmark):\n');
fprintf('  %-22s  %10s  %10s  %8s\n', 'Landmark', 'LL (ms)', 'KD (ms)', 'Speedup');
fprintf('  %s\n', repmat('-', 1, 54));
t_ll_vals = []; t_kd_vals = [];
for k = 1:numKey
    if ~isempty(spec_cache{k})
        c   = spec_cache{k};
        spd = c.t_ll / max(c.t_kd, 1e-9);
        fprintf('  %-22s  %10.4f  %10.4f  %7.1fx\n', landmarks{k}, c.t_ll, c.t_kd, spd);
        t_ll_vals(end+1) = c.t_ll;
        t_kd_vals(end+1) = c.t_kd;
    end
end
fprintf('  %s\n', repmat('-', 1, 54));
if ~isempty(t_ll_vals)
    fprintf('  %-22s  %10.4f  %10.4f  %7.1fx\n', 'average', mean(t_ll_vals), mean(t_kd_vals), mean(t_ll_vals)/max(mean(t_kd_vals),1e-9));
else
    fprintf('  (no landmarks cached)\n');
end

if use_ai
    fprintf('\nGemini extraction vs keyword matching (per sentence):\n');
    fprintf('  %-38s  %10s  %10s  %s\n', 'Sentence', 'Gemini(ms)', 'KW(ms)', 'Status');
    fprintf('  %s\n', repmat('-', 1, 72));
    n_failed = 0;
    for i = 1:length(extract_log)
        e = extract_log{i};
        s = e.sentence;
        if length(s) > 38; s = [s(1:35) '...']; end
        if e.failed
            flag = 'RATE LIMITED (used KW)';
            n_failed = n_failed + 1;
        elseif ~e.match
            flag = 'DIFF';
        else
            flag = 'ok';
        end
        fprintf('  %-38s  %10.2f  %10.4f  %s\n', s, e.t_ai, e.t_kw, flag);
    end
    fprintf('  %s\n', repmat('-', 1, 72));
    n_ok = length(extract_log) - n_failed;
    if n_ok > 0
        fprintf('  Avg Gemini (successful calls): %.2f ms\n', t_ai_total / max(n_ok, 1));
    end
    fprintf('  Avg keyword: %.4f ms\n', t_kw_total / length(conv));
    if n_failed > 0
        fprintf('  %d/%d sentences fell back to keyword matching.\n', n_failed, length(extract_log));
    end
    fprintf('\n  Gemini handles indirect references; keyword matching is faster but literal.\n');
    fprintf('  Extraction latency is hidden during speech pauses.\n');
end

fprintf('\nComplexity summary:\n');
fprintf('  Linked list nearest neighbour: O(K)\n');
fprintf('  KD-Tree nearest neighbour:     O(log K)\n');
fprintf('  Dijkstra (heap):               O((V+E) log V)\n');
fprintf('  Keyword extraction:            O(K*W)  where W = sentence length\n');
fprintf('  Gemini extraction:             O(1) calls, ~500-2000ms network latency\n');
fprintf('  (K=%d key points, V=%d graph nodes)\n', numKey, length(adj));

w_kd          = whos('kd_keys');
bytes_per_node = 32;
ll_est         = numKey * bytes_per_node;
fprintf('\nMemory:\n');
fprintf('  Linked list: ~%d bytes (%d nodes x %d bytes each)\n', ll_est, numKey, bytes_per_node);
fprintf('  KD-Tree:     %d bytes (whos)\n', w_kd.bytes);
fprintf('  KD-Tree overhead vs linked list: ~%.1fx\n', w_kd.bytes / max(ll_est, 1));
fprintf('  Both are O(K) space complexity.\n\n');


%% local functions

function [path, dist] = dijkstra_sp(adj, src, dst, N)
    % min-heap Dijkstra — same algorithm as dijkstra_heap_from_node in map_search_algorithms
    d       = inf(1, N);
    prev    = zeros(1, N);
    d(src)  = 0;
    prev(src) = -1;
    heap    = [src, 0];
    while ~isempty(heap)
        [~, i] = min(heap(:,2));
        u = heap(i,1);
        heap(i,:) = [];
        if u == dst; break; end
        for e = adj{u}
            alt = d(u) + e.w;
            if alt < d(e.to)
                d(e.to)    = alt;
                prev(e.to) = u;
                row = find(heap(:,1) == e.to, 1);
                if isempty(row); heap(end+1,:) = [e.to, alt];
                else;            heap(row,2)   = alt;
                end
            end
        end
    end
    dist = d(dst);
    if isinf(dist); path = []; return; end
    path = dst;
    while prev(path(1)) ~= -1
        path = [prev(path(1)), path];
    end
end

function content = gemini_call(api_key, model, sys_prompt, usr_prompt)
    content = '';
    try
        url  = sprintf('https://generativelanguage.googleapis.com/v1beta/models/%s:generateContent?key=%s', model, api_key);
        full_prompt = sprintf('[SYSTEM]\n%s\n\n[USER]\n%s\n\nIMPORTANT: reply with valid JSON only, no markdown.', sys_prompt, usr_prompt);
        body = struct('contents', {{struct('parts', {{struct('text', full_prompt)}})}}, ...
                      'generationConfig', struct('temperature', 0.7));
        opts     = weboptions('MediaType', 'application/json', 'Timeout', 60, 'RequestMethod', 'post');
        response = webwrite(url, body, opts);

        cands = response.candidates;
        if iscell(cands); c = cands{1}; else; c = cands(1); end
        pts = c.content.parts;
        if iscell(pts); raw = pts{1}.text; else; raw = pts(1).text; end

        raw     = regexprep(raw, '^\s*```[a-z]*\s*', '');
        raw     = regexprep(raw, '\s*```\s*$', '');
        content = strtrim(raw);
    catch err
        fprintf('         [GEMINI] Error: %s\n', err.message);
    end
end

