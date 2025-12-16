import React, { useState, useEffect } from 'react';
import { getCurrentUser, updateProfile } from '../../lib/auth/api';

const UserProfile = ({ user, onProfileUpdate }) => {
  const [formData, setFormData] = useState({
    softwareBackground: user?.softwareBackground || '',
    hardwareBackground: user?.hardwareBackground || ''
  });
  const [loading, setLoading] = useState(false);
  const [success, setSuccess] = useState(false);
  const [error, setError] = useState('');

  useEffect(() => {
    setFormData({
      softwareBackground: user?.softwareBackground || '',
      hardwareBackground: user?.hardwareBackground || ''
    });
  }, [user]);

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
    setSuccess(false);

    try {
      const result = await updateProfile(
        formData.softwareBackground,
        formData.hardwareBackground
      );

      if (result.error) {
        setError(result.error);
      } else {
        setSuccess(true);
        // Update user data in parent component
        onProfileUpdate({
          ...user,
          softwareBackground: formData.softwareBackground,
          hardwareBackground: formData.hardwareBackground
        });

        // Reset success message after 3 seconds
        setTimeout(() => setSuccess(false), 3000);
      }
    } catch (err) {
      setError(err.message || 'An error occurred while updating profile');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="user-profile-container">
      <h3>User Profile</h3>
      <p>Welcome, {user?.name || user?.email}!</p>

      {error && <div className="error-message">{error}</div>}
      {success && <div className="success-message">Profile updated successfully!</div>}

      <form onSubmit={handleSubmit} className="profile-form">
        <div className="form-group">
          <label htmlFor="softwareBackground">Software Background</label>
          <textarea
            id="softwareBackground"
            name="softwareBackground"
            value={formData.softwareBackground}
            onChange={handleChange}
            placeholder="e.g., Programming languages, frameworks, tools you're familiar with..."
            rows="4"
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
            rows="4"
          />
        </div>

        <button type="submit" disabled={loading} className="auth-button">
          {loading ? 'Updating...' : 'Update Profile'}
        </button>
      </form>
    </div>
  );
};

export default UserProfile;