/*! 
    @file     ConfigTree.h
    @brief     Header file for the ConfigTree class. 
    
    @class  ConfigTree
    @brief  Stores configuration data in a tree structure, allowing it to be 
            easily accessed using dot separated paths. The ConfigStorageManager
            saves and loads ConfigTrees from disk. The ConfigManager's 
            interface to the configuration system simply wraps access to a 
            ConfigTree containing the current configuration.


    Currently, boost::property_tree's defaulf 'ptree' is used to store the
    config data.
    Ideally, a basic_ptree with ConfigParameters as its value_type would
    be used instead:
    'boost::property_tree::basic_ptree<std::string, ConfigParameter, std::less<std::string> >'
    Which would allow all conversion of values to be poerformed on loading the 
    tree from disk (instead of converting from/to string with every 
    access/modify).
    Note: It's likely that property tree will eventually have to be replaced
          with something faster.  It looks like ptree's slow string lookups
          using std::vectors<std::string> + std::less<std::string> will be the 
          bottleneck, and replacing it with a faster implementation 
          (using maps + hashing) will be necessary if the config system is to 
          be accessed/updated often.
    
    @author Sophie Calland, Mitchell Metcalfe

  Copyright (c) 2012 Sophie Calland
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ConfigTree_def
#define ConfigTree_def

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <exception>

#include "ConfigParameter.h"
#include "ConfigStorageManager.h"

// using ConfigSystem::ConfigStorageManager;
using boost::property_tree::ptree;

namespace ConfigSystem
{
    class ConfigTree
    {
        public:
        ConfigTree(ptree root);
        ~ConfigTree();
        
        /*! 
         *  @brief  Checks whether the given path and name point to a valid 
         *          existing parameter.
         *  @param paramPath The base path of the parameter.
         *  @param paramName The parameter's name.
         *  @return Returns whether a valid parameter exists.
         */
        bool checkParam (
            const std::string paramPath,
            const std::string paramName
            );

        /*! 
         *  @brief  Gets a parameter from the ConfigTree
         *  @param paramPath The base path of the parameter.
         *  @param paramName The parameter's name.
         *  @param data The parameter to get.
         *  @return Returns whether the operation was successful.
         */
        bool getParam (
            const std::string paramPath,
            const std::string paramName,
            ConfigParameter &data
            );
        
        /*! 
         *  @brief  Stores a parameter into the ConfigTree
         *  @param paramPath The base path of the parameter.
         *  @param paramName The parameter's name.
         *  @param data The parameter to store.
         *  @return Returns whether the operation was successful.
         */
        bool storeParam (
            const std::string paramPath, 
            const std::string paramName, 
            ConfigParameter data
            );
        
        /*! @brief Deletes the named parameter stored at the given path.
         *  @param paramPath Path to the parameter to delete.
         *  @param paramName Name of the parameter to delete.
         *  @return Whether the operation was successful.
         */
        bool DeleteParam(
            const std::string &paramPath,
            const std::string &paramName
            );
        
                                
        //! Return this tree's root node (This method is only intended to
        //! be used by ConfigStorageManager. Modifying the returned ptree
        //! is dangerous).
        ptree getRoot();


    private:
        //! This ConfigTree's root node.
        ptree _treeRoot;
        
        //! The name of the configuration this ConfigTree represents.
        std::string configName;

        /*! 
         *  @brief  Converts a property tree (with the appropriate structure)
         *          into a parameter object.
         *  @param fromPtree    The ptree to convert into a parameter.
         *  @param toParam      The resulting parameter object.
         *  @return Returns whether the conversion succeeded (i.e. if the
         *          minimum set of required fields/keys were not present).
         */
        bool paramFromPtree(ptree fromPtree, ConfigParameter &toParam);

        /*! 
         *  @brief  Converts a parameter object into a property tree that fully
         *          represents the parameter.
         *  @param fromParam  The  parameter object to convert into a ptree.
         *  @param toPtree    The resulting ptree.
         *  @return Returns whether the conversion succeeded (i.e. if the
         *          minimum set of required fields/keys were not present).
         */
        bool ptreeFromParam(ConfigParameter fromParam, ptree &toPtree);
        
        /*!
         *  @brief  Converts a 'conceptual' path into the config 
         *          tree into an actual path that refers to the intended data.
         *  @param paramPath    A path into the config tree.
         *  @return A string containing the full path.
         */
        std::string makeFullPath(
            const std::string paramPath
            );
        /*! 
         *  @brief  Converts a 'conceptual' path and name into the config 
         *          tree into an actual path that refers to the intended data.
         *  @param paramPath    The path to the variable.
         *  @param paramName    The variable's name
         *  @return A string containing the full path.
         */
        std::string makeFullParamPath(
            const std::string paramPath,
            const std::string paramName
            );

        //! Attempts to read parameter information from the given ptree
        //! into the given ConfigParameter.
        //! Returns whether the conversion was successful.
        bool addPtreeValueandRangeToParam(ptree fromPtree, ConfigParameter &toParam);
        
        /*! 
         *  @brief  Attempts to read and convert the "value" from the given
         *          ptree, convert it, and store it in the given parameter
         *  @param fromPtree The ptree from which to read.
         *  @param toParam   The parameter in which to store the converted value.
         *  @return Whether the conversion was successful.
         */
        template<typename T>
        bool addValueToParam(ptree fromPtree, ConfigParameter &toParam)
        {
            T v = fromPtree.get<T>("value");
            if(!toParam.setValue(v)) return false;
            return true;
        }
        
        /*! 
         *  @brief  Attempts to read and convert the given ptree into a vector
         *          of values.
         *  @param from_ptree The ptree, representing a 1d array, from which to read.
         *  @param to_vector  The vector of converted values.
         *  @return Whether the conversion was successful.
         */
        template<typename T>
        bool ptreeToVector1D(
            ptree from_ptree, 
            std::vector<T>& to_vector)
        {
            try
            {
                //Retrieve values of type T from vector in tree and place in vector.
                BOOST_FOREACH(ptree::value_type &child, from_ptree)
                {
                    T val = child.second.get<T>("");
                    to_vector.push_back(val);
                }
            }
            catch(std::exception &e)
            {
                std::cout << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }
        /*! 
         *  @brief  Attempts to read and convert the given ptree into a 2D vector
         *          of values.
         *  @param from_ptree The ptree, representing a 2d array, from which to read.
         *  @param to_vector  The vector of converted values.
         *  @return Whether the conversion was successful.
         */
        template<typename T>
        bool ptreeToVector2D(
            ptree from_ptree, 
            std::vector<std::vector<T> >& to_vector)
        {
            try
            {
                //Retrieve values of type T from vector in tree and place in vector.
                BOOST_FOREACH(ptree::value_type &child, from_ptree)
                {
                    std::vector<T> vec;

                    // ensure child is an array
                    if(child.second.empty()) 
                    {
                        // allow empty arrays
                        if(child.first != "") 
                        {
                            std::cout << __PRETTY_FUNCTION__ << ": " << child.first << " != \"\".";
                            return false;
                        }
                    }
                    else ptreeToVector1D(child.second, vec);

                    to_vector.push_back(std::vector<T>(vec));
                }
            }
            catch(std::exception &e)
            {
                std::cout << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }
        /*! 
         *  @brief  Attempts to read and convert the given ptree into a 3D vector
         *          of values.
         *  @param from_ptree The ptree, representing a 3d array, from which to read.
         *  @param to_vector  The vector of converted values.
         *  @return Whether the conversion was successful.
         */
        template<typename T>
        bool ptreeToVector3D(
            ptree from_ptree, 
            std::vector<std::vector<std::vector<T> > >& to_vector)
        {
            try
            {
                //Retrieve values of type T from vector in tree and place in vector.
                BOOST_FOREACH(ptree::value_type &child, from_ptree)
                {
                    std::vector<std::vector<T> > vec;

                    // ensure child is an array
                    if(child.second.empty()) 
                    {
                        // allow empty arrays
                        if(child.first != "") 
                        {
                            std::cout << __PRETTY_FUNCTION__ << ": " << child.first << " != \"\".";
                            return false;
                        }
                    }
                    else ptreeToVector2D(child.second, vec);

                    to_vector.push_back(std::vector<std::vector<T> >(vec));
                }
            }
            catch(std::exception &e)
            {
                std::cout << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }


        template<typename T>
        bool addVectorValueToParam1D(ptree from_ptree, ConfigParameter &to_param)
        {
            try
            {
                //Retrieve values of type T from vector in tree and place in vector.
                std::vector<T> vector_value;
                ptreeToVector1D(from_ptree.get_child("value"), vector_value);

                //Sets the value in the ConfigParameter object
                return to_param.setValue(vector_value);
            }
            catch(std::exception &e)
            {
                std::cout << "ERROR: " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }
        template<typename T>
        bool addVectorValueToParam2D(ptree from_ptree, ConfigParameter &to_param)
        {
            try
            {
                //Retrieve values of type T from vector in tree and place in vector.
                std::vector< std::vector<T> > vector_value;
                ptreeToVector2D(from_ptree.get_child("value"), vector_value);
                
                //Sets the value in the ConfigParameter object
                return to_param.setValue(vector_value);
            }
            catch(std::exception &e)
            {
                std::cout << "ERROR: " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }
        template<typename T>
        bool addVectorValueToParam3D(ptree from_ptree, ConfigParameter &to_param)
        {
            try
            {
                //Retrieve values of type T from vector in tree and place in vector.
                std::vector< std::vector< std::vector<T> > > vector_value;
                ptreeToVector3D(from_ptree.get_child("value"), vector_value);
                
                //Sets the value in the ConfigParameter object
                return to_param.setValue(vector_value);
            }
            catch(std::exception &e)
            {
                std::cout << "ERROR: " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }
        

        template<typename T>
        bool addRangeToParam(ptree fromPtree, ConfigParameter &toParam)
        {
            // Read range
            std::string lBStr = fromPtree.get("range.lBound", "none");
            std::string uBStr = fromPtree.get("range.uBound", "none");
            BoundType range_lBound = stringToBoundType(lBStr);
            BoundType range_uBound = stringToBoundType(uBStr);

            std::string outsideStr = fromPtree.get("range.outside", "false");
            bool range_outside  = (outsideStr.compare("true") == 0);

            std::string autoClipStr = fromPtree.get("range.autoClip", "false");
            bool range_autoClip  = (autoClipStr.compare("true") == 0);

            T range_min = fromPtree.get("range.min", 0.0);
            T range_max = fromPtree.get("range.max", 0.0);
            ConfigRange<T> cr
            (
                range_min,
                range_max,
                range_outside,
                range_autoClip,
                range_lBound,
                range_uBound
            );
            
            return toParam.setRange(cr);
        }


        /*! 
         *  @brief  Creates a ptree representing the given parameter.
         *  @param fromParam The parameter to convert.
         *  @param toPtree   The ptree representing the given parameter.
         *  @return Whether the conversion was successful.
         */
        bool addParamValueandRangeToPtree(ConfigParameter fromParam, ptree &toPtree);
        
        // Note: should explicitly specify template params + put implementations in .cpp
        template<typename T>
        bool addValueToPtree(ConfigParameter fromParam, ptree &toPtree)
        {
            T v; 
            if(!fromParam.getValue(&v)) return false;
            toPtree.put("value", v);
            return true;
        }
        
        template<typename T>
        bool addVectorValueToPtree1D(ConfigParameter from_param, ptree &to_ptree)
        {
            ptree children;
            ptree child;
            std::vector<T> retrieved_vector;
            //Retrieve vector from the CP object
            if( !from_param.getValue(&retrieved_vector) ) return false;
            
            try
            {
                BOOST_FOREACH(const T &value, retrieved_vector)
                {
                    child.put("", value);
                    children.push_back(std::make_pair("", child));
                }
            
                to_ptree.add_child("value", children);
            }
            catch(std::exception &e)
            {
                std::cout << "ERROR: " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }
        template<typename T>
        bool addVectorValueToPtree2D(ConfigParameter from_param, ptree &to_ptree)
        {
            ptree child;
            ptree grandchild;
            ptree greatgrandchild;
            std::vector< std::vector<T> > retrieved_vector;
            
            //Retrieve vector from the CP object
            if(!from_param.getValue(&retrieved_vector))
            {
                std::cout << "addVectorValueToPtree2D(...): No 2d vector in this ConfigParameter."
                          << std::endl;
                return false;
            }

            try
            {
                BOOST_FOREACH(const std::vector<T> &column, retrieved_vector)
                {
                    grandchild = ptree();
                    greatgrandchild = ptree();
                    
                    BOOST_FOREACH(const T &value, column)
                    {
                        greatgrandchild.put("", value);
                        grandchild.push_back(std::make_pair("", greatgrandchild));
                    }
                    
                    child.push_back(std::make_pair("", grandchild));
                }
            
                to_ptree.add_child("value", child);
            }
            catch(std::exception &e)
            {
                std::cout << "ERROR: " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }
        template<typename T>
        bool addVectorValueToPtree3D(ConfigParameter from_param, ptree &to_ptree)
        {
            ptree parent;
            ptree child;
            ptree grandchild;
            ptree greatgrandchild;
            std::vector< std::vector< std::vector<T> > > retrieved_vector;
            
            //Retrieve vector from the CP object
            if( !from_param.getValue(&retrieved_vector) ) return false;
            
            try
            {
                BOOST_FOREACH(const std::vector< std::vector<T> > &column, retrieved_vector)
                {
                    child = ptree();
                    grandchild = ptree();
                    greatgrandchild = ptree();
                    
                    BOOST_FOREACH(const std::vector<T> &column2, column)
                    {
                        grandchild = ptree();
                        greatgrandchild = ptree();
                        
                        BOOST_FOREACH(const T &value, column2)
                        {
                            greatgrandchild.put("", value);
                            grandchild.push_back(std::make_pair("", greatgrandchild));
                        }
                        
                        child.push_back(std::make_pair("", grandchild));
                    }
                    
                    parent.push_back(std::make_pair("", child));
                }
            
                to_ptree.add_child("value", parent);
            }
            catch(std::exception &e)
            {
                std::cout << "ERROR: " << e.what() << std::endl;
                return false;
            }
            
            return true;
        }
        
        
        template<typename T>
        bool addRangeToPtree(ConfigParameter fromParam, ptree &toPtree)
        {
            ConfigRange<T> r;
            if(!fromParam.getRange(&r)) return false;

            toPtree.put("range.min"     , r.getMin());
            toPtree.put("range.max"     , r.getMax());

            std::string lBStr = makeBoundTypeString(r.getLowerBoundType());
            std::string uBStr = makeBoundTypeString(r.getUpperBoundType());
            toPtree.put("range.lBound"  , lBStr);
            toPtree.put("range.uBound"  , uBStr);
            
            std::string outsideStr = (r.getOutside())? "true" : "false";
            toPtree.put("range.outside" , outsideStr);

            std::string autoClipStr = (r.getAutoClip())? "true" : "false";
            toPtree.put("range.autoClip", autoClipStr);

            return true;
        }
    };
}

#endif
