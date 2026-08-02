// Simplified Steam Deck controller schematic, built as SVG. Each interactive
// part gets id="part-<role>" + class "deck-part" so app.js can toggle the
// "active" class from live Gamepad API state. Geometry is a rough
// approximation for a status schematic, not a pixel-accurate render.
function buildDeckSchematic(svg) {
  const ns = 'http://www.w3.org/2000/svg';
  function el(tag, attrs) {
    const e = document.createElementNS(ns, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    return e;
  }
  function part(role, tag, attrs) {
    const e = el(tag, Object.assign({ id: 'part-' + role, class: 'deck-part' }, attrs));
    svg.appendChild(e);
    return e;
  }
  function label(text, x, y) {
    const t = el('text', { x, y, class: 'deck-label' });
    t.textContent = text;
    svg.appendChild(t);
  }

  // Body outline (static, not highlightable).
  svg.appendChild(el('rect', {
    x: 10, y: 30, width: 380, height: 170, rx: 26,
    fill: '#ffffff', stroke: '#d5d5d5', 'stroke-width': 1.5,
  }));

  // Left trackpad + stick.
  part('trackpadL', 'rect', { x: 34, y: 56, width: 62, height: 52, rx: 8 });
  label('L-PAD', 65, 46);
  part('stickL', 'circle', { cx: 65, cy: 150, r: 22 });
  label('L-STICK', 65, 182);

  // Right trackpad + stick.
  part('trackpadR', 'rect', { x: 304, y: 56, width: 62, height: 52, rx: 8 });
  label('R-PAD', 335, 46);
  part('stickR', 'circle', { cx: 300, cy: 150, r: 22 });
  label('R-STICK', 300, 182);

  // ABXY diamond.
  part('btnY', 'circle', { cx: 250, cy: 100, r: 10 });
  part('btnX', 'circle', { cx: 228, cy: 122, r: 10 });
  part('btnB', 'circle', { cx: 272, cy: 122, r: 10 });
  part('btnA', 'circle', { cx: 250, cy: 144, r: 10 });
  label('ABXY', 250, 165);

  // D-pad.
  part('dpadUp', 'rect', { x: 142, y: 96, width: 12, height: 14 });
  part('dpadDown', 'rect', { x: 142, y: 128, width: 12, height: 14 });
  part('dpadLeft', 'rect', { x: 126, y: 112, width: 14, height: 12 });
  part('dpadRight', 'rect', { x: 158, y: 112, width: 14, height: 12 });
  label('D-PAD', 148, 155);

  // Bumpers + triggers.
  part('l2', 'rect', { x: 30, y: 2, width: 50, height: 10, rx: 3 });
  part('r2', 'rect', { x: 320, y: 2, width: 50, height: 10, rx: 3 });
  label('L2/L4', 55, 10); label('R2/R4', 345, 10);
  part('l1', 'rect', { x: 30, y: 16, width: 50, height: 14, rx: 4 });
  part('r1', 'rect', { x: 320, y: 16, width: 50, height: 14, rx: 4 });
  label('L1', 55, 34); label('R1', 345, 34);

  // Select / Start / Home.
  part('select', 'rect', { x: 178, y: 60, width: 18, height: 10, rx: 3 });
  part('start', 'rect', { x: 204, y: 60, width: 18, height: 10, rx: 3 });
  part('home', 'circle', { cx: 200, cy: 90, r: 9 });
  label('HOME', 200, 108);
}
