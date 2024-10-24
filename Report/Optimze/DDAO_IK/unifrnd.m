function rnd = unifrnd (a, b, r, c)

  if (nargin > 1)
    if (~isscalar(a) || ~isscalar(b)) 
      [retval, a, b] = common_size (a, b);
      if (retval > 0)
	error ('unifrnd: a and b must be of common size or scalar');
      end
    end
  end

  if (nargin == 4)
    if (~ (isscalar (r) && (r > 0) && (r == round (r))))
      error ('unifrnd: r must be a positive integer');
    end
    if (~ (isscalar (c) && (c > 0) && (c == round (c))))
      error ('unifrnd: c must be a positive integer');
    end
    sz = [r, c];

    if (any (size (a) ~= 1) && (length (size (a)) ~= length (sz) || any (size (a) ~= sz)))
      error ('unifrnd: a and b must be scalar or of size [r, c]');
    end
  elseif (nargin == 3)
    if (isscalar (r) && (r > 0))
      sz = [r, r];
    elseif (isvector(r) && all (r > 0))
      sz = r(:)';
    else
      error ('unifrnd: r must be a positive integer or vector');
    end

    if (any (size (a) ~= 1)	&& (length (size (a)) ~= length (sz) || any (size (a) ~= sz)))
      error ('unifrnd: a and b must be scalar or of size sz');
    end
  elseif (nargin == 2)
    sz = size(a);
  else
    print_usage ();
  end

  if (isscalar(a) && isscalar(b))
    if (find (~(-Inf < a) | ~(a < b) | ~(b < Inf)))
      rnd = NaN * ones(sz);
    else
      rnd =  a + (b - a) .* rand (sz);
    end
  else
    rnd =  a + (b - a) .* rand (sz);

    k = find (~(-Inf < a) | ~(a < b) | ~(b < Inf));
    if (any (k))
      rnd(k) = NaN;
    end
  end

end