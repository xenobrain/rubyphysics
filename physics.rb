class Physics
  attr_accessor :draw_debug

  def initialize
    @frame_step = 1.0 / 60.0
    @velocity_iterations = 4
    @position_iterations = 2

    @h = @frame_step / @velocity_iterations
    @inv_h = 1.0 / @h

    @hertz = 60.0

    @bounds = []
    @bodies = []

    @contacts = {}
    @constraints = []

    @draw_debug = true

    @gravity_x = 0.0
    @gravity_y = -9.8
  end

  def tilemap=(grid)
    @grid = grid[:grid]
    @grid_w = grid[:grid_w]
    @grid_h = grid[:grid_h]
    @grid_t = grid[:grid_size]
    @grid_ht = @grid_t * 0.5
    @grid_s = Math.log2(@grid_t).to_i
  end

  def tick
    find_collisions
    prepare_contacts_soft

    i = 0
    while i < @velocity_iterations
      fn.each_send(@bodies, self, :integrate_velocity)
      warm_start_contacts
      #tgs_soft(true)

      fn.each_send(@bodies, self, :integrate_position)
      #tgs_soft(false)
      i += 1
    end

    fn.each_send(@bodies, self, :finalize_position)
  end

  def new_aabb(x, y, w, h, density = 1.0, dynamic = true)
    m = w * h * density
    i = m * (w * w + h * h) / 12.0

    box = {
      type: :aabb,
      bounds: {
        x: x,
        y: y,
        w: w,
        h: h,
        min_x: 0,
        min_y: 0,
        max_x: w,
        max_y: h,
      }
    }

    box[:bounds][:shape] = box

    {
      # Position
      x: x,
      y: y,
      # Rotation
      qs: 0.0,
      qc: 0.0,
      # Velocity
      vx: 0.0,
      vy: 0.0,
      vw: 0.0,
      # Masses
      inv_m: 1.0 / m,
      inv_i: 1.0 / i,

      friction: 0.5,

      torque: 0.0,
      force_x: 0.0,
      force_y: 0.0,
      gravity_scale: 1.0,
      linear_damping: 1.0,
      angular_damping: 1.0,


      shape: box,
      type: dynamic ? :body_dynamic : :body_static
    }
  end

  def new_circle(x, y, r, density = 1.0, dynamic = true)
    {}
  end

  def new_capsule(x, y, w, h, density = 1.0, dynamic = true)
    {}
  end

  def new_segment(x, y, x2, y2, density = 1.0, dynamic = true)
    {}
  end

  def new_polygon(x, y, vertices, density = 1.0, dynamic = true)
    num_vertices = vertices.length

    # Compute aabb, area, centroid, and moment of inertia
    min_x = Float::INFINITY
    max_x = -Float::INFINITY
    min_y = Float::INFINITY
    max_y = -Float::INFINITY
    inv_i = 0.0
    area = 0.0
    cx = 0.0
    cy = 0.0

    i = 0
    while i < num_vertices
      x1, y1 = vertices[i]
      min_x = x1 if x1 < min_x
      max_x = x1 if x1 > max_x
      min_y = y1 if y1 < min_y
      max_y = y1 if y1 > max_y

      j = (i + 1) % num_vertices
      x2, y2 = vertices[j]

      area += x1 * y2 - x2 * y1
      cx += (x1 + x2) * (x1 * y2 - x2 * y1)
      cy += (y1 + y2) * (x1 * y2 - x2 * y1)
      inv_i += (x1 * x1 + x2 * x2) * (y1 - y2)

      i += 1
    end

    area = area.abs * 0.5
    cx /= 6.0 * area
    cy /= 6.0 * area
    inv_m = 1.0 / (area * density)
    inv_i *= 1.0 / (12.0 * density * inv_m)

    # Offset the hull around the centroid
    i = 0
    while i < num_vertices
      vertices[i][0] -= cx
      vertices[i][1] -= cy
      i += 1
    end

    polygon = {
      type: :polygon,
      vertices: vertices,
      triangles: Triangles.earclip(vertices.flatten),
      bounds: {
        x: x,
        y: y,
        w: max_x,
        h: max_y,
        min_x: min_x,
        min_y: min_y,
        max_x: max_x,
        max_y: max_y,
      }
    }

    polygon[:bounds][:shape] = polygon

    {
      # Position
      x: x,
      y: y,

      # Rotation
      qs: 0.0,
      qc: 0.0,

      # Velocity
      vx: 0.0,
      vy: 0.0,
      vw: 0.0,

      # Masses
      inv_m: inv_m,
      inv_i: inv_i,

      shape: polygon,
      type: dynamic ? :body_dynamic : :body_static
    }
  end

  def add(body)
    @bodies << body
    @bounds << body[:shape][:bounds]
  end

  def remove(body)
    @bodies.delete(body)
    @bounds.delete(body[:shape][:bounds])
  end

  # Performs a ray cast against an axis-aligned bounding box (AABB).
  # Adapted from "Real-Time Collision Detection," p. 179.
  #
  # @param a [Hash] The AABB represented as a hash with keys `:x` (min x-coordinate) and `:w` (width).
  # @param p1x [Float] The x-coordinate of the ray's starting point.
  # @param p1y [Float] The y-coordinate of the ray's starting point.
  # @param p2x [Float] The x-coordinate of the ray's ending point.
  # @param p2y [Float] The y-coordinate of the ray's ending point.
  # @return [Hash] A hash containing the result of the ray cast, with keys:
  #   - `:hit` [Boolean]: Whether the ray intersects the AABB.
  #   - `:fraction` [Float, nil]: The fraction along the ray's direction where the intersection occurs.
  #   - `:nx` [Float]: The x-component of the normal at the intersection point.
  #   - `:ny` [Float]: The y-component of the normal at the intersection point.
  #   - `:x` [Float, nil]: The x-coordinate of the intersection point.
  #   - `:y` [Float, nil]: The y-coordinate of the intersection point.
  def ray_cast_aabb(a, p1x, p1y, p2x, p2y)
    output = {
      hit: false,
      fraction: nil,
      nx: 0.0,
      ny: 0.0,
      x: nil,
      y: nil
    }

    t_min = -Float::INFINITY
    t_max = Float::INFINITY

    px = p1x
    py = p1y
    dx = p2x - p1x
    dy = p2y - p1y

    abs_dx = dx.abs
    abs_dy = dy.abs

    # x-coordinate
    if abs_dx < Float::EPSILON
      # parallel
      if px < a[:x] || px > a[:x] + a[:w]
        return output
      end
    else
      inv_d = 1.0 / dx
      t1 = (a[:x] - px) * inv_d
      t2 = (a[:x] + a[:w] - px) * inv_d

      s = -1.0
      if t1 > t2
        t1, t2 = t2, t1
        s = 1.0
      end

      # Push the min up
      if t1 > t_min
        output[:nx] = s
        output[:ny] = 0.0
        t_min = t1
      end

      # Pull the max down
      t_max = [t_max, t2].min

      return output if t_min > t_max
    end

    # y-coordinate
    if abs_dy < Float::EPSILON
      # parallel
      if py < a[:y] || py > a[:h]
        return output
      end
    else
      inv_d = 1.0 / dy
      t1 = (a[:y] - py) * inv_d
      t2 = (a[:y] + a[:h] - py) * inv_d

      s = -1.0
      if t1 > t2
        t1, t2 = t2, t1
        s = 1.0
      end

      # Push the min up
      if t1 > t_min
        output[:nx] = 0.0
        output[:ny] = s
        t_min = t1
      end

      # Pull the max down
      t_max = [t_max, t2].min

      return output if t_min > t_max
    end

    # Does the ray start inside the box?
    # Does the ray intersect beyond the max fraction?
    return output if t_min < 0.0 || t_min > 1.0

    # Intersection.
    output[:fraction] = t_min
    output[:x] = p1x + t_min * dx
    output[:y] = p1y + t_min * dy
    output[:hit] = true
    output
  end

  def ray_cast_aabb2(ray_x, ray_y, ray_dx, ray_dy, ray_length, aabb_x, aabb_y, aabb_w, aabb_h)
    result = {
      hit: false,
      fraction: nil,
      nx: 0.0,
      ny: 0.0,
      x: nil,
      y: nil
    }

    p0_x, p0_y = ray_x, ray_y
    p1_x = ray_x + ray_dx * ray_length
    p1_y = ray_y + ray_dy * ray_length

    # Create AABB bounds for the ray segment
    aabb_min_x = [p0_x, p1_x].min
    aabb_max_x = [p0_x, p1_x].max
    aabb_min_y = [p0_y, p1_y].min
    aabb_max_y = [p0_y, p1_y].max

    aabb_box_min_x = aabb_x
    aabb_box_max_x = aabb_x + aabb_w
    aabb_box_min_y = aabb_y
    aabb_box_max_y = aabb_y + aabb_h

    # Test AABB overlap
    return result if aabb_max_x < aabb_box_min_x || aabb_min_x > aabb_box_max_x ||
      aabb_max_y < aabb_box_min_y || aabb_min_y > aabb_box_max_y

    # Check ray's intersection with the AABB planes
    def ray_to_plane(ray_start, ray_dir, min_bound, max_bound)
      if ray_dir == 0
        return ray_start < min_bound || ray_start > max_bound ? Float::INFINITY : 0
      else
        t1 = (min_bound - ray_start) / ray_dir
        t2 = (max_bound - ray_start) / ray_dir
        t1, t2 = t2, t1 if t1 > t2
        return t1, t2
      end
    end

    t_min, t_max = 0.0, ray_length

    t1_min, t1_max = ray_to_plane(p0_x, ray_dx, aabb_box_min_x, aabb_box_max_x)
    t2_min, t2_max = ray_to_plane(p0_y, ray_dy, aabb_box_min_y, aabb_box_max_y)

    t_min = [t_min, t1_min, t2_min].max
    t_max = [t_max, t1_max, t2_max].min

    return result if t_min > t_max

    # Calculate intersection point
    result[:fraction] = t_min
    result[:x] = p0_x + ray_dx * t_min
    result[:y] = p0_y + ray_dy * t_min
    result[:hit] = true

    # Determine normal
    if t1_max >= t_min
      result[:nx] = ray_dx < 0 ? -1.0 : 1.0
      result[:ny] = 0.0
    else
      result[:nx] = 0.0
      result[:ny] = ray_dy < 0 ? -1.0 : 1.0
    end

    result
  end

  def ray_intersects_aabb(ray_x, ray_y, ray_dx, ray_dy, ray_length, aabb_x, aabb_y, aabb_w, aabb_h)
    # Calculate the AABB bounds
    aabb_min_x = aabb_x
    aabb_max_x = aabb_x + aabb_w
    aabb_min_y = aabb_y
    aabb_max_y = aabb_y + aabb_h

    t_min = 0.0
    t_max = ray_length

    # Check intersection with vertical planes (left and right)
    if ray_dx != 0
      t1 = (aabb_min_x - ray_x) / ray_dx
      t2 = (aabb_max_x - ray_x) / ray_dx

      if t1 > t2
        t1, t2 = t2, t1
      end

      t_min = [t_min, t1].max
      t_max = [t_max, t2].min

      if t_min > t_max
        return nil # No intersection
      end
    else
      # Ray is parallel to vertical planes
      if ray_x < aabb_min_x || ray_x > aabb_max_x
        return nil # No intersection
      end
    end

    # Check intersection with horizontal planes (top and bottom)
    if ray_dy != 0
      t1 = (aabb_min_y - ray_y) / ray_dy
      t2 = (aabb_max_y - ray_y) / ray_dy

      if t1 > t2
        t1, t2 = t2, t1
      end

      t_min = [t_min, t1].max
      t_max = [t_max, t2].min

      if t_min > t_max
        return nil # No intersection
      end
    else
      # Ray is parallel to horizontal planes
      if ray_y < aabb_min_y || ray_y > aabb_max_y
        return nil # No intersection
      end
    end

    # Compute the intersection point
    intersection_x = ray_x + ray_dx * t_min
    intersection_y = ray_y + ray_dy * t_min

    [intersection_x, intersection_y]
  end

  def raycast_aabb_grid(p1x, p1y, p2x, p2y)
    # Convert world coordinates to tile coordinates
    start_tile_x = (p1x / @grid_t).to_i
    start_tile_y = (p1y / @grid_t).to_i
    end_tile_x = (p2x / @grid_t).to_i
    end_tile_y = (p2y / @grid_t).to_i

    tx = start_tile_x
    while tx <= end_tile_x
      ty = start_tile_y
      while ty <= end_tile_y
        index = ty * @grid_w + tx
        if @grid[index] == 1  # Collidable tile
          # Tile boundaries in world space
          tile_x = tx * @grid_t
          tile_y = ty * @grid_t

          # Raycast against the tile
          result = ray_cast_aabb(
            { x: tile_x, y: tile_y, w: @grid_t, h: @grid_t },
            p1x, p1y, p2x, p2y
          )

          return result if result[:hit]
        end

        ty += 1
      end

      tx += 1
    end

    { hit: false, fraction: nil, nx: 0.0, ny: 0.0, x: nil, y: nil }
  end

  def raycast_mouse_direction_grid(p1x, p1y, ray_length = 1280)
    ray_length
    mouse_x = GTK.args.state.cursor_x + GTK.args.state.camera.x
    mouse_y = GTK.args.state.cursor_y + GTK.args.state.camera.y

    # Direction of the ray
    dx = mouse_x - p1x
    dy = mouse_y - p1y
    length = Math.sqrt(dx * dx + dy * dy)
    direction_x = dx / length
    direction_y = dy / length

    # Calculate the endpoint using the fixed ray length
    p2x = p1x + direction_x * ray_length
    p2y = p1y + direction_y * ray_length

    # Convert start point to tile coordinates
    current_x = p1x
    current_y = p1y
    tx = (current_x / @grid_t).to_i
    ty = (current_y / @grid_t).to_i

    step_x = direction_x > 0 ? 1 : -1
    step_y = direction_y > 0 ? 1 : -1

    t_max_x = ((tx + (step_x > 0 ? 1 : 0)) * @grid_t - current_x) / direction_x
    t_max_y = ((ty + (step_y > 0 ? 1 : 0)) * @grid_t - current_y) / direction_y
    t_delta_x = @grid_t.abs / direction_x.abs
    t_delta_y = @grid_t.abs / direction_y.abs

    while (current_x - p1x).abs < ray_length && (current_y - p1y).abs < ray_length
      index = ty * @grid_w + tx
      if @grid[index] == 1
        tile_x = tx * @grid_t
        tile_y = ty * @grid_t

        result = ray_cast_aabb(
          { x: tile_x, y: tile_y, w: @grid_t, h: @grid_t },
          p1x, p1y, p2x, p2y
        )

        return result if result[:hit]
      end

      if t_max_x < t_max_y
        tx += step_x
        current_x += (t_max_x * direction_x)
        current_y += (t_max_x * direction_y)
        t_max_x += t_delta_x
      else
        ty += step_y
        current_x += (t_max_y * direction_x)
        current_y += (t_max_y * direction_y)
        t_max_y += t_delta_y
      end

      break if tx < 0 || ty < 0 || tx >= @grid_w || ty >= @grid_h
    end

    { hit: false, fraction: nil, nx: 0.0, ny: 0.0, x: nil, y: nil }
  end

  def calc_circle_circle(a, b) end

  def calc_circle_capsule(a, b) end

  def calc_circle_segment(a, b) end

  def calc_circle_polygon(a, b) end

  def calc_capsule_capsule(a, b) end

  def calc_capsule_segment(a, b) end

  def calc_capsule_polygon(a, b) end

  def calc_segment_segment(a, b) end

  def calc_segment_polygon(a, b) end

  def calc_polygon_polygon(a, b) end

  def calc_aabb_grid(a)
    return unless a[:shape][:type] == :aabb

    bounds = a[:shape][:bounds]
    min_x = (bounds[:x].to_i >> @grid_s).clamp(0, @grid_w - 1)
    min_y = (bounds[:y].to_i >> @grid_s).clamp(0, @grid_h - 1)
    max_x = ((bounds[:x] + bounds[:w]).to_i >> @grid_s).clamp(0, @grid_w - 1)
    max_y = ((bounds[:y] + bounds[:h]).to_i >> @grid_s).clamp(0, @grid_h - 1)

    c_x_a = bounds[:x] + bounds[:w] * 0.5
    c_y_a = bounds[:y] + bounds[:h] * 0.5
    e_a = bounds[:w] * 0.5
    e_b = bounds[:h] * 0.5

    x = min_x
    while x <= max_x
      y = min_y
      while y <= max_y
        contact_id = a.object_id ^ (x * 3344921057) ^ (y * 3344921057)
        contact = @contacts[contact_id]

        if @grid[y * @grid_w + x] == 1
          if @draw_debug
            camera_x = $gtk.args.state.camera[:x]
            camera_y = $gtk.args.state.camera[:y]
            $gtk.args.outputs.borders << { x: x * @grid_t - camera_x, y: y * @grid_t - camera_y, w: @grid_t, h: @grid_t, g: 255 }
          end
          cxb = x * @grid_t + @grid_ht
          cyb = y * @grid_t + @grid_ht
          d0x = cxb - c_x_a
          d0y = cyb - c_y_a

          dx = e_a + @grid_ht - d0x.abs
          if dx < 0
            y += 1
            next
          end

          dy = e_b + @grid_ht - d0y.abs
          if dy < 0
            y += 1
            next
          end

          if dx < dy
            depth = dx
            if d0x < 0
              nx = -1.0
              ny = 0.0
              px = c_x_a - e_a
              py = c_y_a
            else
              nx = 1.0
              ny = 0.0
              px = c_x_a + e_a
              py = c_y_a
            end
          else
            depth = dy
            if d0y < 0
              nx = 0.0
              ny = -1.0
              px = c_x_a
              py = c_y_a - e_b
            else
              nx = 0.0
              ny = 1.0
              px = c_x_a
              py = c_y_a + e_b
            end
          end

          # Transform contact point into local space of `a`
          local_x_a = a[:qc] * (px - c_x_a) + a[:qs] * (py - c_y_a)
          local_y_a = -a[:qs] * (px - c_x_a) + a[:qc] * (py - c_y_a)

          # For `b` (assuming it’s a static grid tile with identity transform)
          local_x_b = px - (x * @grid_t)
          local_y_b = py - (y * @grid_t)

          b = { x: x * @grid_t, y: y * @grid_t, qs: 0.0, qc: 0.0, vx: 0.0, vy: 0.0, vw: 0.0, inv_m: 0.0, inv_i: 0.0, friction: 0.5 }
          if contact
            # update contact, persisted is set to true
            # Currently there is no warm starting
            friction = Math.sqrt(a[:friction] * b[:friction])
            @contacts[contact_id] = { a: a, b: b, points: [{ x: px, y: py, local_x_a: local_x_a, local_y_a: local_y_a, local_x_b: local_x_b, local_y_b: local_y_b, n_impulse: 0.0, t_impulse: 0.0, bias: 0.0 }], nx: nx, ny: ny, depth: depth, friction: friction, persisted: false }
          else
            friction = Math.sqrt(a[:friction] * b[:friction])
            @contacts[contact_id] = { a: a, b: b, points: [{ x: px, y: py, local_x_a: local_x_a, local_y_a: local_y_a, local_x_b: local_x_b, local_y_b: local_y_b,  n_impulse: 0.0, t_impulse: 0.0, bias: 0.0 }], nx: nx, ny: ny, depth: depth, friction: friction, persisted: false }
          end
        else
          @contacts.delete(contact_id) # TODO: optimize this so it doesn't need to be done every tile
        end
        y += 1
      end
      x += 1
    end
  end

  def find_collisions
    fn.each_send(@bodies, self, :calc_aabb_grid) if @grid
    Geometry.each_intersect_rect(@bounds, @bounds) do |a, b| end
  end

  def tgs_soft(use_bias)
    @contacts.each_value do |contact|
      num_points = contact[:points].length
      points = contact[:points]

      u = contact[:friction]
      nx = contact[:nx]
      ny = contact[:ny]

      tx = -ny
      ty = nx

      a = contact[:a]
      b = contact[:b]

      m_a = a[:inv_m]
      i_a = a[:inv_i]
      m_b = b[:inv_m]
      i_b = b[:inv_i]

      v_x_a = a[:vx]
      v_y_a = a[:vy]
      v_x_b = b[:vx]
      v_y_b = b[:vy]
      w_a = a[:vw]
      w_b = b[:vw]

      i = 0
      while i < num_points
        point = points[i]

        local_x_a = point[:local_x_a]
        local_y_a = point[:local_y_a]
        local_x_b = point[:local_x_b]
        local_y_b = point[:local_y_b]

        # Relative velocity
        r_vx = v_x_b - v_x_a + w_b * local_y_b - w_a * local_y_a
        r_vy = v_y_b - v_y_a - w_b * local_x_b + w_a * local_x_a

        # Compute normal impulse
        vn = r_vx * nx + r_vy * ny
        p = -point[:n_mass] * (vn + (use_bias ? point[:bias] : 0.0))
        point[:n_impulse] = [point[:n_impulse] + p, 0.0].max

        # Apply the normal impulse
        px = p * nx
        py = p * ny

        v_x_a -= px * m_a
        v_y_a -= py * m_a
        v_x_b += px * m_b
        v_y_b += py * m_b
        w_a -= (local_x_a * py - local_y_a * px) * i_a
        w_a += (local_x_b * py - local_y_b * px) * i_b

        # Recompute relative velocity
        r_vx = v_x_b - v_x_a + w_b * local_y_b - w_a * local_y_a
        r_vy = v_y_b - v_y_a - w_b * local_x_b + w_a * local_x_a

        # Compute tangential impulse
        vt = r_vx * tx + r_vy * ty
        p = vt * point[:t_mass]
        max_u = u * point[:n_impulse]
        new_p = [[point[:t_impulse] + p, -max_u].max, max_u].min
        p = new_p - point[:t_impulse]
        point[:t_impulse] = new_p
        px = p * tx
        py = p * ty

        # Apply the tangential impulse
        v_x_a -= px * m_a
        v_y_a -= py * m_a
        v_x_b += px * m_b
        v_y_b += py * m_b
        w_a -= (local_x_a * py - local_y_a * px) * i_a
        w_b += (local_x_b * py - local_y_b * px) * i_b

        i += 1
      end

      a[:vx] = v_x_a
      a[:vy] = v_y_a
      b[:vx] = v_x_b
      b[:vy] = v_y_b
      a[:vw] = w_a
      b[:vw] = w_b
    end
  end

  def warm_start_contacts
    @contacts.each_value do |contact|
      a = contact[:a]
      b = contact[:b]

      # Body properties
      m_a = a[:inv_m]
      i_a = a[:inv_i]
      m_b = b[:inv_m]
      i_b = b[:inv_i]

      # Body velocities
      v_x_a = a[:vx]
      v_y_a = a[:vy]
      v_x_b = b[:vx]
      v_y_b = b[:vy]
      w_a = a[:vw]
      w_b = b[:vw]

      # Body rotations
      q_s_a = a[:qs]
      q_c_a = a[:qc]
      q_s_b = b[:qs]
      q_c_b = b[:qc]

      # Normal and tangent vectors
      nx = contact[:nx]
      ny = contact[:ny]
      tx = -ny
      ty =  nx

      points = contact[:points]
      num_points = points.length
      i = 0

      while i < num_points
        point = points[i]

        local_x_a = point[:local_x_a]
        local_y_a = point[:local_y_a]
        local_x_b = point[:local_x_b]
        local_y_b = point[:local_y_b]

        r_x_a = q_c_a * local_x_a - q_s_a * local_y_a
        r_y_a = q_s_a * local_x_a + q_c_a * local_y_a
        r_x_b = q_c_b * local_x_b - q_s_b * local_y_b
        r_y_b = q_s_b * local_x_b + q_c_b * local_y_b

        # Warm start impulses
        n_impulse = point[:n_impulse]
        t_impulse = point[:t_impulse]

        # Calculate the total impulse
        px = n_impulse * nx + t_impulse * tx
        py = n_impulse * ny + t_impulse * ty

        # Apply impulses to body A
        w_a -= i_a * (r_x_a * py - r_y_a * px)
        v_x_a -= m_a * px
        v_y_a -= m_a * py

        # Apply impulses to body B
        w_b += i_b * (r_x_b * py - r_y_b * px)
        v_x_b += m_b * px
        v_y_b += m_b * py

        i += 1
      end

      a[:vx] = v_x_a
      a[:vy] = v_y_a
      b[:vx] = v_x_b
      b[:vy] = v_y_b
      a[:vw] = w_a
      b[:vw] = w_b
    end
  end

  def prepare_contacts_soft
    @contacts.each_value do |contact|
      x_a = contact[:a][:x]
      y_a = contact[:a][:y]
      x_b = contact[:b][:x]
      y_b = contact[:b][:y]

      m_a = contact[:a][:inv_m]
      i_a = contact[:a][:inv_i]
      m_b = contact[:b][:inv_m]
      i_b = contact[:b][:inv_i]

      # Determine contact hertz based on presence of mass
      contact_hertz = (m_a == 0.0 || m_b == 0.0) ? 2.0 * @hertz : @hertz

      zeta = 10.0
      omega = 2.0 * Math::PI / contact_hertz
      c = @h * omega * (2.0 * zeta + @h * omega)

      # Rotation for body A
      q_s_a = contact[:a][:qs]
      q_c_a = contact[:a][:qc]

      # Rotation for body B
      q_s_b = contact[:b][:qs]
      q_c_b = contact[:b][:qc]

      nx = contact[:nx]
      ny = contact[:ny]
      tx = -ny
      ty =  nx

      points = contact[:points]
      num_points = points.length
      i = 0

      while i < num_points
        point = points[i]

        x_a = point[:local_x_a] -= x_a
        y_a = point[:local_y_a] -= y_a
        x_b = point[:local_x_b] -= x_b
        y_b = point[:local_y_b] -= y_b

        # Rotate local anchor points
        r_x_a = q_c_a * x_a - q_s_a * y_a
        r_y_a = q_s_a * x_a + q_c_a * y_a
        r_x_b = q_c_b * x_b - q_s_b * y_b
        r_y_b = q_s_b * x_b + q_c_b * y_b

        # Store the rotated anchor points
        point[:r_x_a0] = r_x_a
        point[:r_y_a0] = r_y_a
        point[:r_x_b0] = r_x_b
        point[:r_y_b0] = r_y_b

        point[:depth] = contact[:depth]
        point[:adjusted_depth] = point[:depth]

        # Calculate the mass in the normal direction
        r_na = r_x_a * nx + r_y_a * ny
        r_nb = r_x_b * nx + r_y_b * ny
        k_n = m_a + m_b + i_a + r_na * r_na + i_b + r_nb * r_nb
        point[:n_mass] = k_n > 0.0 ? 1.0 / k_n : 0.0

        # Calculate the mass in the tangent direction
        r_ta = r_x_a * tx + r_y_a * ty
        r_tb = r_x_b * tx + r_y_b * ty
        k_tangent = m_a + m_b + i_a * r_ta * r_ta + i_b * r_tb * r_tb
        point[:t_mass] = k_tangent > 0.0 ? 1.0 / k_tangent : 0.0

        # Soft contact parameters
        point[:impulse_coeff] = 1.0 / (1.0 + c)
        point[:mass_coeff] = c * point[:impulse_coeff]
        point[:bias_coeff] = omega / (2.0 * zeta + @h * omega)

        i += 1
      end
    end
  end

  def solve_contacts

  end

  def integrate_position(body)
    body[:x] += body[:vx]
    body[:y] += body[:vy]

    bounds = body[:shape][:bounds]

    min_x = body[:x]
    min_y = body[:y]

    # Velocity expansion
    min_x += body[:vx] if body[:vx] < 0
    min_y += body[:vy] if body[:vy] < 0
    bounds[:x] = min_x
    bounds[:y] = min_y
    bounds[:w] = bounds[:max_x] + body[:vx].abs
    bounds[:h] = bounds[:max_y] + body[:vy].abs
  end

  def integrate_velocity(body)
    return unless body[:type] == :body_dynamic

    inv_m = body[:inv_m]
    inv_i = body[:inv_i]
    vx = body[:vx]
    vy = body[:vy]
    vw = body[:vw]

    # Integrate linear velocity
    vx += @h * inv_m * (body[:force_x] + body[:gravity_scale] * @gravity_x)
    vy += @h * inv_m * (body[:force_y] + body[:gravity_scale] * @gravity_y)

    # Integrate angular velocity
    vw += @h * inv_i * body[:torque]

    # Apply damping
    vx /= (1.0 + @h * body[:linear_damping])
    vy /= (1.0 + @h * body[:linear_damping])
    vw /= (1.0 + @h * body[:angular_damping])

    # Update velocities in the body hash
    body[:vx] = vx
    body[:vy] = vy
    body[:vw] = vw
  end

  def finalize_position(body)
    if @draw_debug
      bounds = body[:shape][:bounds]
      camera_x = $gtk.args.state.camera[:x]
      camera_y = $gtk.args.state.camera[:y]
      $gtk.args.outputs.borders << { x: bounds[:x] + (bounds[:w] * 0.5) - camera_x, y: bounds[:y] + (bounds[:h] * 0.5) - camera_y, w: bounds[:w], h: bounds[:h], r: 255, anchor_x: 0.5, anchor_y: 0.5 }
    end

    # Zero the forces
    body[:force_x] = 0
    body[:force_y] = 0
    body[:torque] = 0
  end
end

$physics = Physics.new
