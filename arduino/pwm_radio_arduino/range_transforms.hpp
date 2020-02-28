template<typename T>
class Range3Levels {
public:
  T low;
  T zero;
  T high;
  Range3Levels(T low_, T zero_, T high_): low(low_), zero(zero_), high(high_){}
};



template <typename T>
T interp(T value, T in_min, T in_max, T out_min, T out_max) {
  auto interpolated = out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min);
  return constrain(interpolated, out_min, out_max);
}

int transform(float input, const Range3Levels<float>& range_in, const Range3Levels<float>& range_out) {
   if (input >= range_in.zero) {
    return interp(input, range_in.zero, range_in.high, range_out.zero, range_out.high);
   }
   return interp(input, range_in.low, range_in.zero, range_out.low, range_out.zero);
}
