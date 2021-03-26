classdef PlotStyler < matlab.mixin.Copyable
    %PLOTSTYLER Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        PENNCOLORS = {'#01256E', ...
                       '#95001A', ...
                       '#F2C100', ...
                       '#008E00', ...
                       '#C35A00', ...
                       '#4A0042'};
        PENNDARK = {'#000F3A', ...
                    '#57000A', ...
                    '#AF7F00', ...
                    '#005200', ...
                    '#812D00', ...
                    '#23001F'};
        PENNGRAYSCALE = {'#131315', ...
                         '#44464B', ...
                         '#6C6F76', ...
                         '#A8AAAF', ...
                         '#CFD0D2', ...
                         '#F2F2F3'};
        COLORS = [PlotStyler.PENNCOLORS, PlotStyler.PENNDARK, PlotStyler.PENNGRAYSCALE];
        PENNCOLORNAMES = {'b', 'r', 'y', 'g', 'o', 'p'};
        PENNDARKCOLORNAMES = {'db', 'dr', 'dy', 'dg', 'do', 'dp'};
        PENNGRAYNAMES = {'d2', 'd1', 'm2', 'm1', 'l2', 'l1'};
        COLORNAMES = [PlotStyler.PENNCOLORNAMES, PlotStyler.PENNDARKCOLORNAMES, PlotStyler.PENNGRAYNAMES];
        AXISCOLOR = PlotStyler.PENNGRAYSCALE{2}
        ALLSTYLES = {'-',':','--','-.'};
        SOLID = {'-'};
        MEDIUMLINEWIDTH = 4;
        HEIGHT = 500;
        ASPECT = 1;
        INTERPRETER = 'Latex';
        LATEXFONTSIZE = 18;
        POINTMARKERSIZE = 300;
        ARROWSCALE = 0.4;
        MINARROWLENGTH = 1e-1;
        TEXT_POS_CODES = {'b', 't', 'l', 'r', 'bl', 'br', 'tl', 'tr'};
        TEXT_HOR = {'center', 'center', 'right', 'left', 'right', 'left', 'right', 'left'};
        TEXT_VERT = {'top', 'bottom', 'middle', 'middle', 'top', 'top', 'bottom', 'bottom'};
        TEXT_MARGIN = [0 0 -1 1 -1 1 -1 1; -1 1 0 0 -1 -1 1 1];
        LABELALIGNMENT = 't';
        ARROWALIGNMENT = 't';
        LABELMARGIN = 0.0;
        LATEXLABELFONTMULTIPLIER = 1.8;
        LATEXTITLEFONTMULTIPLIER = 2;
        LATEXDIAGRAMFONTMULTIPLIER = 3;
        ABS_PADDING = 1e-4;
        REL_PADDING = 0.1;
        PRINTIMAGEEXTENSION = '.png'; % '.eps';
        PRINTIMAGETYPE = 'png'; % 'epsc';
        
    end
    
    properties
        fig
        axesList
        height = PlotStyler.HEIGHT
        aspect = PlotStyler.ASPECT
        interpreter = PlotStyler.INTERPRETER;
        colors = PlotStyler.PENNCOLORS;
        linestyles = PlotStyler.ALLSTYLES;
        linewidth = PlotStyler.MEDIUMLINEWIDTH;
        fontsize = PlotStyler.LATEXFONTSIZE;
        arrowscale = PlotStyler.ARROWSCALE;
    end
    
    methods
        function obj = PlotStyler(varargin)
            obj = obj.attach(varargin{:});
        end
        
        function obj = attach(obj, fig, ax)
            % create figure if one is not supplied
            if nargin < 2
                fig = figure;
            end
            
            obj.fig = fig;
            
            % create axes if one is not supplied;
            if nargin < 3
                al = findall(fig,'type','axes');
                
                % generate axes if the figure is blank
                if numel(al) < 1
                    axes;
                end
                obj.axesList = findall(fig,'type','axes');
            else
                obj.axesList(1) = ax;
            end
            
            obj.applyAll();
        end
        
        function obj = close(obj)
            close(obj.fig);
        end
        
        function obj = print(obj, filename)
            figure(obj.fig);
            savefile = [filename PlotStyler.PRINTIMAGEEXTENSION];
            fprintf("Saving image %s\n", savefile)
            print(savefile, ...
                ['-d' PlotStyler.PRINTIMAGETYPE]);
        end
        
        function set.height(obj, h)
            obj.height = h;
            obj.applySize();
        end
        
        function set.aspect(obj, a)
            obj.aspect = a;
            obj.applySize();
        end
        
        function set.interpreter(obj, i)
            obj.interpreter = i;
            obj.applyInterpreter();
        end

        function set.colors(obj, c)
            obj.colors = c;
            obj.applyColors();
        end
        
        function set.linestyles(obj, ls)
            obj.linestyles = ls;
            obj.applyLineStyles();
        end
        
        function set.linewidth(obj, lw)
            obj.linewidth = lw;
            obj.applyLineWidth();
        end
        
        function set.fontsize(obj, fs)
            obj.fontsize = fs;
            obj.applyInterpreter();
        end
        
        function set.arrowscale(obj, as)
            obj.arrowscale = as;
        end
        
        function applySize(obj)
            bounds = [0 0, ...
                      round(obj.height*obj.aspect), ...
                      obj.height];
            set(obj.fig,'Position',bounds);
        end
        
        function applyInterpreter(obj)
            f = obj.fig;
            interp = obj.interpreter;
            
            % set defaults
            set(f, 'defaultTextInterpreter', interp);
            set(f, 'defaultLegendInterpreter', interp);
            set(f, 'defaultAxesTickLabelInterpreter', interp)
            
            % override existing
            interpretable = findall(f,'-property','Interpreter');
            set(interpretable,'Interpreter', interp);
            set(obj.axesList,'TickLabelInterpreter', interp);
            
            % set text default sizes
            set(f, 'defaultTextFontSize', obj.fontsize);
            if strcmpi(interp,'Latex')
                set(f, 'DefaultAxesFontSize', obj.fontsize);
                set(f, 'DefaultAxesTitleFontSizeMultiplier', PlotStyler.LATEXTITLEFONTMULTIPLIER);
                set(f, 'DefaultAxesLabelFontSizeMultiplier', PlotStyler.LATEXLABELFONTMULTIPLIER);
            end
        end
        
        function applyColors(obj)
            co = obj.colors;
            figure(obj.fig);
            for ax = obj.axesList
                axes(ax);
                colororder(co);
            end
        end
        
        function applyLineStyles(obj)
            ls = obj.linestyles;
            figure(obj.fig);
            for ax = obj.axesList
                axes(ax);
                ax.LineStyleOrder = ls;
            end
        end
        
        function applyLineWidth(obj)
            f = obj.fig;
            lw = obj.linewidth;
            
            % set default
            set(f, 'defaultLineLineWidth', lw);
            
            % override existing
            set(findall(f, 'Type', 'Line'), 'LineWidth', lw);
            set(findall(f, 'Type', 'Quiver'), 'LineWidth', lw);
        end
        
        function applyAll(obj)
            obj.applySize();
            obj.applyInterpreter();
            obj.applyColors();
            obj.applyLineStyles();
            obj.applyLineWidth();
        end
        
        
        function plotWithBars(~, x, y, dy, varargin)
            ax = gca;
            hold on;
            filling_alpha = 0.3;
            
            % make it vertical
            if size(x,1) == 1
                x = x(:);
            end
            
            if size(y,1) == 1
                y = y(:);
            end
            
            if size(dy,1) == 1
                dy = dy(:);
            end
            
            % broadcast
            x = zeros(size(y)) + x;
            
            xfill = [x; flipud(x)];
            yfill = [y - dy; flipud(y + dy)];
            for i=1:size(xfill,2)
                line = plot(x(:,i),y(:,i),varargin{:});
                c = line.Color;
                filling = fill(xfill(:,i), yfill(:,i), c, ...
                    'linestyle', 'none', 'HandleVisibility', 'off');
                ax.ColorOrderIndex = ax.ColorOrderIndex-1;
                alpha(filling, filling_alpha);
            end
        end
        
        function sc = plotPoint(obj, point, txt, color, rot, alignment)
            scatargs = {};
            if nargin > 3
               color = PlotStyler.colorComponents(color);
               scatargs = {color};
            end
            scatargs{end + 1} = 'filled';
            sc = scatter(point(1,:), point(2,:), ...
                PlotStyler.POINTMARKERSIZE, scatargs{:});

            % TODO: multiple points support
            if nargin >= 3 && numel(point) == 2
                
                if nargin < 5
                    rot = 0;
                end
                
                if nargin < 6
                    alignment = PlotStyler.LABELALIGNMENT;
                end
                
                obj.labelPoint(point, txt, sc.CData, rot, alignment);

            end
        end
        
        function q = plotArrow(obj, points, arrows, txt, color, alignment)
            anargs = {'LineWidth', obj.linewidth};
            color = PlotStyler.colorComponents(color);
            anargs{end+1} = 'Color';
            anargs{end+1} = color;
            
            if nargin < 4
                txt = '';
            end
            
            if nargin < 6
                alignment = PlotStyler.ARROWALIGNMENT;
            end
            
            deltas = arrows * obj.arrowscale;
            small_norms = vecnorm(deltas) < 1e-8;
            if any(small_norms)
                if ~strcmp(txt, '')
                    if txt(1) == '$'
                        txt = [txt(1:end-1) '= 0$'];
                    %else
                    %    txt = [txt(1:end) '= 0'];
                    end
                end
                obj.plotPoint(points(:, small_norms), txt, color, 0.0, 't');
            end
            
            points = points(:, ~small_norms);
            arrows = arrows(:, ~small_norms);
            q = quiver(points(1, :), points(2, :), ...
                arrows(1, :), arrows(2, :), ...
                obj.arrowscale, anargs{:});
            q.MaxHeadSize = 1;

            if ~strcmp(txt, '') && any(~small_norms)
                [text_alignment, tex_toffset, text_rot] = ...
                    PlotStyler.parseArrowText(deltas, alignment);
                text_point = tex_toffset + points;
                obj.labelPoint(text_point, txt, q.Color, text_rot, ...
                    text_alignment);
            end
            

        end
        
        function plotSpan(obj, point1, point2, txt, color, alignment)
            line_args = {'LineWidth', obj.linewidth};
            color = PlotStyler.colorComponents(color);
            line_args{end+1} = 'Color';
            line_args{end+1} = color;
            
            if nargin < 4
                txt = '';
            end
            
            if nargin < 6
                alignment = 'c';
            end
            
            delta = point2 - point1;
            
            normal = [delta(2) -delta(1)]' / norm(delta);
            bar = norm(delta) / 10 * [-normal normal];
            bars{1} = point2 + bar;
            bars{2} = point1 + bar;
            bars{3} = [point1 point2] + 1/15 * [delta -delta];
            for i=1:length(bars)
                bar = bars{i};
                line(bar(1, :), bar(2, :), line_args{:});
            end
            

            if ~strcmp(txt, '')
                [text_alignment, tex_toffset, text_rot] = ...
                    PlotStyler.parseArrowText(delta, alignment);
                text_point = tex_toffset + point1;
                obj.labelPoint(text_point, txt, color, text_rot, ...
                    text_alignment);
            end
            

        end
        
        function labelPoint(~, point, txt, color, rot, alignment, txtmult)
            if nargin < 5
                    rot = 0;
            end
            if nargin < 6
                alignment = PlotStyler.LABELALIGNMENT;
            end
            if nargin < 7
                txtmult = PlotStyler.LATEXDIAGRAMFONTMULTIPLIER;
            end
            [al, margin] = PlotStyler.parseAlignment(alignment);
            margin = margin .* [1 0.25]';
            point = point + PlotStyler.LABELMARGIN * margin;
            if numel(txt) > 0 && ~(txt(1) == '$')
               txt = ['\bf ' txt];
            end
            h = text(point(1), point(2), txt, al{:});
            % upgrade text size
            set(h, 'FontSize', get(h, 'FontSize') * txtmult);
            set(h, 'FontWeight', 'bold');
            set(h, 'Rotation', rot);
            set(h, 'Color', color);
        end
        
        function axisStyle(obj, char_array)
            for ax = obj.axesList
                axes(ax);
                eval(['axis ', char_array]);
            end
        end
        
        function grid(obj, minor)
            if nargin < 2
                minor = false;
            end
            
            for ax = obj.axesList
                axes(ax);
                grid on;
                gc = PlotStyler.hex2rgb(PlotStyler.PENNGRAYSCALE{1});
                ax.GridColor = gc;
                if minor
                    grid minor;
                    mgc = PlotStyler.hex2rgb(PlotStyler.PENNGRAYSCALE{2});
                    ax.MinorGridColor = mgc;
                    %ax.MinorGridAlpha = ax.GridAlpha;
                    %ax.MinorGridLineStyle = '-';
                end
            end
        end
        
        function box(obj, setting)
            set(obj.axesList,'Box', setting);
        end
        
        function scaling(obj, xsc, ysc)
            assert(strcmp(xsc,'linear') || strcmp(xsc,'log'));
            assert(strcmp(ysc,'linear') || strcmp(ysc,'log'));
            for ax=obj.axesList
                set(ax, 'XScale', xsc);
                set(ax, 'YScale', ysc);
            end
        end
        
        function axisLimit(obj, bbox, rel_pad, abs_pad)            
            % extend axes past plot edge
            if nargin < 3
                rel_pad = PlotStyler.REL_PADDING;
            end
            
            if nargin < 4
                abs_pad = PlotStyler.ABS_PADDING;
            end
            
            for ax = obj.axesList
                axes(ax);
                xl = [min(bbox(:,1)) max(bbox(:,2))];
                yl = [min(bbox(:,3)) max(bbox(:,4))];
                xlim(PlotStyler.padAxis(xl, rel_pad, abs_pad));
                ylim(PlotStyler.padAxis(yl, rel_pad, abs_pad)); 
            end
        end
        
        function originToLimits(obj, headed, rel_pad, abs_pad, dirs)
            
            % put arrowheads on axes
            if nargin < 2
                headed = false;
            end
            
            % which directions to go from origin?
            % [up down left right]
            if nargin < 5
                dirs = true(1, 4);
            end
            
            % extend axes past plot edge
            if nargin < 3
                rel_pad = PlotStyler.REL_PADDING;
            end
            if nargin < 5
                abs_pad = PlotStyler.ABS_PADDING;
            end
            
            zs = zeros(1,2);
            lw = obj.linewidth/2;
            g = PlotStyler.hex2rgb(PlotStyler.AXISCOLOR);
            head_portion = .05;
            head_aspect = 1/3;
            
            for ax = obj.axesList
                axes(ax);
                xl = PlotStyler.padAxis(xlim(ax), rel_pad, abs_pad);
                yl = PlotStyler.padAxis(ylim(ax), rel_pad, abs_pad);
                halfaxes_x = [zs; zs; 0 xl(1); 0 xl(2)];
                halfaxes_y = [0 yl(2); 0 yl(1); zs; zs];
                for i = 1:4
                    if dirs(i)
                        ha = plot(halfaxes_x(i, :), halfaxes_y(i, :), ...
                            '-', 'Color', g, 'LineWidth', lw);
                        uistack(ha,'bottom');
                    end
                end
                if headed
                    % assume axis square
                    % construct arrowhead shape
                    min_axis_length = min(xl(2)-xl(1), yl(2)-yl(1));
                    hl = min_axis_length * head_portion;
                    hw = hl * head_aspect;
                    arrow = [-hw, hw,  0;
                                0, 0, hl];
                    
                    % translate and rotate to plot edges
                    R90 = [0 -1; 1 0];
                    top = arrow + [0; yl(2) - hl];
                    bottom = -arrow + [0; yl(1) + hl];
                    left = R90 * arrow + [xl(1) + hl; 0];
                    right = -R90 * arrow + [xl(2) - hl; 0];
                    
                    % plot arrowheads
                    X = [top(1,:); bottom(1,:); left(1,:); right(1,:)]';
                    Y = [top(2,:); bottom(2,:); left(2,:); right(2,:)]';
                    X = X(:, dirs);
                    Y = Y(:, dirs);
                    arrowheads = fill(X, Y, g, ...
                        'LineWidth', lw, ...
                        'LineStyle', '-', ...
                        'EdgeColor', g);
                    uistack(arrowheads,'bottom');
                end
            end
        end
        
        function asCurve(obj, rel_pad, abs_pad)
            if nargin < 2
                rel_pad = PlotStyler.REL_PADDING;
            end
            if nargin < 3
                abs_pad = PlotStyler.ABS_PADDING;
            end
            obj.grid(true);
            obj.box('on');
            obj.originToLimits(false, rel_pad, abs_pad);
            obj.axisStyle('tight');
        end
        
        function asSeries(obj)
            obj.grid(true);
            obj.box('on');
            obj.axisStyle('tight');
        end
        
        function asDoodle(obj, dirs)
            if nargin < 2
                dirs = true(1, 4);
            end
            obj.axisStyle('equal off');
            obj.originToLimits(true, PlotStyler.REL_PADDING, ...
                PlotStyler.ABS_PADDING, dirs);
        end
        
        function obj = asFreeBodyDiagram(obj)
            obj.linewidth = PlotStyler.MEDIUMLINEWIDTH * 1.5;
            polys = findall(obj.fig, 'Type', 'Polygon');
            for p=polys(:)'
                fc = p.FaceColor;
                darkened = PlotStyler.darken(p.FaceColor);
                p.EdgeColor = darkened;
                p.LineWidth = obj.linewidth;
                p.FaceAlpha = 1;
            end
        end
        
    end

    methods(Static)
        function paxis = padAxis(lims, rel_pad, abs_pad)
            if nargin < 3
                abs_pad = 0.0;
            end
            scale = lims(2) - lims(1);
            
            paxis = lims + (rel_pad * scale + abs_pad) * [-1 1];
        end
        
        function rgb = hex2rgb(c)
            hexString = regexprep(c,'[^a-fA-f0-9]',''); 
            if size(hexString,2) == 3
                r = double(hex2dec(hexString(1)))/255;
                g = double(hex2dec(hexString(2)))/255;
                b = double(hex2dec(hexString(3)))/255;
            elseif size(hexString,2) == 6
                r = double(hex2dec(hexString(1:2)))/255;
                g = double(hex2dec(hexString(3:4)))/255;
                b = double(hex2dec(hexString(5:6)))/255;

            else
                error('invalid hex color code!');
            end
            rgb = [r, g, b];
        end
        
        function c = colorComponents(c)
           if isa(c,'char')
               if numel(c) <= 2
                   c = PlotStyler.colorkey(c);
               else
                   c = PlotStyler.hex2rgb(c);
               end
           elseif ~(size(c, 1) == 1 && size(c, 2) == 3)
               error('color illegible!');
           end
        end
        
        function dc = darken(c)
            ci = PlotStyler.getColorIndex(c);
            if ci <= 6
                % color; shift to dark colors
                ci = ci + 6;
            elseif ci >= 13 && ci <= 18
                % greyscale; subtract one
                ci = ci - 1;
            end
            dcn = PlotStyler.COLORNAMES{ci};
            dc = PlotStyler.colorComponents(dcn);
        end
        
        function i = getColorIndex(c)
            if isa(c,'char')
                if numel(c) <= 2
                    i = PlotStyler.colorIdxFromKey(c);
                else
                    i = PlotStyler.colorIdxFromHex(c);
                end
            elseif (size(c, 1) == 1 && size(c, 2) == 3)
                i = PlotStyler.colorIdxFromComponents(c);
            else
                error('color illegible!');
            end
            
        end
        
        function i = colorIdxFromComponents(c)
           for i = 1:length(PlotStyler.COLORNAMES)
                ci = PlotStyler.colorComponents(PlotStyler.COLORNAMES{i});
                if norm(c - ci) < 1e-6
                    break
                end
            end 
        end
        
        function i = colorIdxFromKey(c)
           i = [];
           for j=1:length(PlotStyler.COLORS)
               if strcmp(PlotStyler.COLORNAMES{j}, c)
                   i = j;
               end
           end
           if numel(i) ~= 1
               error('color illegible!');
           end
        end
        
        function i = colorIdxFromHex(c)
           i = [];
           for j=1:length(PlotStyler.COLORS)
               if strcmp(PlotStyler.COLORS{j}, c)
                   i = j;
               end
           end
           if numel(i) ~= 1
               error('color illegible!');
           end
        end
        
        function col = colorkey(c)
           i = PlotStyler.colorIdxFromKey(c);
           col = PlotStyler.hex2rgb(PlotStyler.COLORS{i});
        end
        
        function [al, textoffset, rot] = parseArrowText(delta, pos)
           if ~isa(pos, 'char')
               error('only char typed alignments allowed!')
           end
           
           rot = atand(delta(2)/delta(1));
           up_dir = [-sin(rot); cos(rot)];
           
           switch pos
               case 'h'
                   al = PlotStyler.closestAlignment(delta);
                   margin = [1 0]';
                   rot = 0;
               case 'c'
                   al = 't';
                   margin = [0 1]';
               case 't'
                   al = PlotStyler.closestAlignment(-delta);
                   margin = [-1 0]';
                   rot = 0;
               otherwise
                   error('unrecognized arrow position code!');
           end
           margin = margin .* [1/2 1/16]';
           margin = norm(delta) * margin;
           textoffset = delta/2 + [delta/norm(delta)  up_dir] * margin;
        end
        
        function al = closestAlignment(dir)
            % match direction to margin via cosine similarity
            margins = PlotStyler.TEXT_MARGIN;
            diffs = atan2(margins(2,:), margins(1,:)) ...
                - atan2(dir(2), dir(1));
            [~, i] = min(abs(mod(diffs, 2 * pi)));
            i = i(1);
            al = PlotStyler.TEXT_POS_CODES{i};
        end
        
        function [opts, margin] = parseAlignment(al)
            h = '';
            v = '';
            if ~isa(al, 'char')
               error('only char typed alignments allowed!')
            end

            found = false;
            
            for i = 1:length(PlotStyler.TEXT_POS_CODES)
                if strcmp(PlotStyler.TEXT_POS_CODES{i}, al)
                    found = true;
                    h = PlotStyler.TEXT_HOR{i};
                    v = PlotStyler.TEXT_VERT{i};
                    margin = PlotStyler.TEXT_MARGIN(:,i);
                end
            end
            
            if ~found
                error('unrecognized position code!');
            end
            opts = {'horizontalAlignment', h, 'verticalAlignment', v};
        end
    end
end

