#include <map>
#include <exception>

template <class KeyClass, class ValueClass>
class InterpolatingMap
{
public:
    InterpolatingMap() { };
    
    void insert (KeyClass key, ValueClass value) 
    {
        internal_map[key] = value;
    }
    
    ValueClass lookup(KeyClass key)
    {
        if (internal_map.empty())
        {
            throw std::runtime_error("Interpolating map lookup called but map is empty!!");
        }
        typename std::map<KeyClass, ValueClass>::iterator i = internal_map.lower_bound(key);
        if (i == internal_map.end())
        {
            typename std::map<KeyClass, ValueClass>::iterator j = i;
            j--;
            return (*j).second;
        }
        if (i == internal_map.begin())
        {
            return (*(internal_map.begin())).second;
        }
        typename std::map<KeyClass, ValueClass>::iterator j = i;
        j--;
        KeyClass interpolation = (key - (*j).first) / ((*i).first - (*j).first);
        ValueClass result = (interpolation * ((*i).second - (*j).second)) + (*j).second;
        return result;
    }

    bool empty()
    {
        return internal_map.empty();
    }
private:
    std::map<KeyClass, ValueClass> internal_map;
};