import React, { createContext, useContext, useState, useEffect } from 'react';
import { getCurrentUser } from '../../lib/auth/api';

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const initAuth = async () => {
      const token = localStorage.getItem('better-auth-session-token');
      if (token) {
        try {
          const result = await getCurrentUser();
          if (result.success) {
            setUser(result.user);
          } else {
            // Token might be invalid, remove it
            localStorage.removeItem('better-auth-session-token');
          }
        } catch (error) {
          console.error('Auth initialization error:', error);
          localStorage.removeItem('better-auth-session-token');
        }
      }
      setLoading(false);
    };

    initAuth();
  }, []);

  const login = (userData) => {
    setUser(userData);
  };

  const logout = () => {
    setUser(null);
    localStorage.removeItem('better-auth-session-token');
  };

  const updateUser = (userData) => {
    setUser(userData);
  };

  const value = {
    user,
    loading,
    login,
    logout,
    updateUser
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};