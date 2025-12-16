import React, { useState } from 'react';
import { signup } from '../../lib/auth/api';

const SignupForm = ({ onSignupSuccess }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    softwareBackground: '',
    hardwareBackground: ''
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const result = await signup(
        formData.email,
        formData.password,
        formData.name,
        formData.softwareBackground,
        formData.hardwareBackground
      );

      if (result.error) {
        setError(result.error);
      } else {
        // Store session token in localStorage
        localStorage.setItem('better-auth-session-token', result.session.token);
        onSignupSuccess(result.user);
      }
    } catch (err) {
      setError(err.message || 'An error occurred during signup');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2>Create Your Account</h2>
      {error && <div className="error-message">{error}</div>}

      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="name">Full Name</label>
          <input
            type="text"
            id="name"
            name="name"
            value={formData.name}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="softwareBackground">Software Background</label>
          <textarea
            id="softwareBackground"
            name="softwareBackground"
            value={formData.softwareBackground}
            onChange={handleChange}
            placeholder="e.g., Programming languages, frameworks, tools you're familiar with..."
            rows="3"
          />
        </div>

        <div className="form-group">
          <label htmlFor="hardwareBackground">Hardware Background</label>
          <textarea
            id="hardwareBackground"
            name="hardwareBackground"
            value={formData.hardwareBackground}
            onChange={handleChange}
            placeholder="e.g., Experience with robotics, electronics, sensors, etc."
            rows="3"
          />
        </div>

        <button type="submit" disabled={loading} className="auth-button">
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;