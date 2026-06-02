function add_event_label(ax, xval, lab, side)
%ADD_EVENT_LABEL Place a boxed label for vertical event at xval on axes ax
% Usage: add_event_label(ax, xval, 'Attack start', 'left')
    if nargin < 4 || isempty(side), side = 'right'; end
    try
        if isempty(ax) || ~isgraphics(ax, 'axes'), ax = gca; end
        axes(ax); %#ok<LAXES>
        xl = xlim(ax); yl = ylim(ax);
        rng = xl(2) - xl(1);
        % horizontal offset: 2% of axis width
        dx = rng * 0.02;
        % y position slightly below top with breathing room
        y = yl(2) - 0.08 * (yl(2) - yl(1));
        if strcmpi(side,'left')
            x = xval - dx;
            hal = 'right';
        else
            x = xval + dx;
            hal = 'left';
        end
        % clamp x into axis with small margin
        margin = max(0.005 * rng, eps);
        x = min(max(x, xl(1) + margin), xl(2) - margin);
        t = text(ax, x, y, lab, 'HorizontalAlignment', hal, 'VerticalAlignment', 'top', ...
            'FontSize', 11, 'FontWeight', 'normal', 'Color', [0 0 0], 'Interpreter', 'none');
        set(t, 'BackgroundColor', [1 1 1], 'EdgeColor', 0.85*[1 1 1], 'Margin', 2);
    catch
        % fail silently to avoid breaking plotting
    end
end
