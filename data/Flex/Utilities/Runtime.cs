using System;
using System.Collections.Generic;
using System.Linq;

namespace Flex.Utilities;

public static class Runtime
{
    // TODO Turn bool into enum flag
    // TODO: Create a function to get and create an array of types
    
    public static IEnumerable<Type> GetAllTypes<T>(bool isClass = true,bool notAbstract = true) where T : class
    {
        return AppDomain.CurrentDomain.GetAssemblies().SelectMany(a =>
            a.GetTypes().Where(t =>
                (t.IsClass && isClass) && (!t.IsAbstract && notAbstract) && t.IsSubclassOf(typeof(T))));
        
    }

    /// <summary>
    /// Create class from type
    /// </summary>
    /// <param name="type"> The class type</param>
    /// <param name="constructorArgs"> The constructor args if theres any</param>
    /// <typeparam name="T"> Cast to type </typeparam>
    /// <returns></returns>
    public static T CreateClassFromType<T>(Type type,params object[] constructorArgs)
    {
        return (T)Activator.CreateInstance(type, constructorArgs)!;
    }
    
    /// <summary>
    /// Create class from type
    /// </summary>
    /// <param name="constructorArgs">The constructor args if theres any </param>
    /// <typeparam name="T"> The base class </typeparam>
    /// <returns></returns>
    public static T CreateClassFromType<T>(params object[] constructorArgs)
    {
        return CreateClassFromType<T>(typeof(T), constructorArgs);
    }
}